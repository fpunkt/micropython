/* The MIT License (MIT)
 *
 * Copyright (c) 2021 fpunkt@me.com
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*
This driver is based on
*/

// #if MICROPY_HW_ENABLE_CAN

#include <string.h>

#include "py/obj.h"
#include "py/objarray.h"
#include "py/binary.h"
#include "py/runtime.h"
#include "py/builtin.h"
#include "py/mphal.h"
#include "py/mperrno.h"
#include "mpconfigport.h"

// ESP-IDF includes
#include "soc/dport_reg.h"
#include "driver/twai.h"
#include "esp_err.h"
#include "esp_log.h"

#include "modmachine.h"

#define DEVICE_NAME "CAN"


// Check for errors
// Allow passing of function calls (argument is evaluated only once)
#define ESP_CHECK_ERROR(status) { \
    esp_err_t e = status; \
    if (e != ESP_OK) mp_raise_OSError(-abs(e)); \
}

typedef enum filter_mode_e {
    FILTER_RAW_SINGLE = 0,
    FILTER_RAW_DUAL,
    FILTER_ADDRESS
} filter_mode_t;

typedef struct machine_can_config_s {
    twai_timing_config_t timing;
    twai_filter_config_t filter;
    twai_general_config_t general;
    uint16_t baudrate;
    bool initialized;
} machine_can_config_t;

typedef struct machine_can_obj_s {
    mp_obj_base_t base;
    machine_can_config_t *config;
    mp_obj_t rxcallback;
    TaskHandle_t receiver_task;
    mp_obj_t received_object;
    mp_obj_t *received_object_items;
    byte rx_state;
    bool extframe : 1;
    bool loopback : 1;
    uint16_t num_error_warning; //FIXME: populate this value somewhere
    uint16_t num_error_passive;
    uint16_t num_bus_off;
} machine_can_obj_t;

typedef enum rx_state_e {
    RX_STATE_FIFO_EMPTY = 0,
    RX_STATE_MESSAGE_PENDING,
    RX_STATE_FIFO_FULL,
    RX_STATE_FIFO_OVERFLOW,
} rx_state_t;

// TWAI_TIMING_CONFIG_125KBITS()   {.brp = 32, .tseg_1 = 15, .tseg_2 = 4, .sjw = 3, .triple_sampling = false}
#define CAN_DEFAULT_PRESCALER (32)
#define CAN_DEFAULT_SJW (3)
#define CAN_DEFAULT_BS1 (15)
#define CAN_DEFAULT_BS2 (4)

// forward refrerences
static machine_can_obj_t machine_can_obj;
mp_obj_t machine_hw_can_make_new(const mp_obj_type_t *type, size_t n_args, size_t n_kw, const mp_obj_t *args);
STATIC mp_obj_t machine_hw_can_init_helper(machine_can_obj_t *self, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args);
STATIC void machine_hw_can_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind);

STATIC twai_status_info_t _machine_hw_can_get_status() {
    twai_status_info_t status;
    uint32_t err_code = twai_get_status_info(&status);
    if (err_code != ESP_OK) {
        mp_raise_OSError(-err_code);
    }
    return status;
}

STATIC void _machine_hw_can_set_filter(machine_can_obj_t *self, uint32_t addr, uint32_t mask, uint8_t bank, bool rtr) {
    if (bank > ((self->extframe && self->config->filter.single_filter) ? 0 : 1 )) {
        mp_raise_ValueError("CAN filter parameter error");
    }
    uint32_t preserve_mask;
    if (self->extframe) {
        addr = (addr & 0x1FFFFFFF) << 3 | (rtr ? 0x04 : 0);
        mask = (mask & 0x1FFFFFFF) << 3 | 0x03;
        preserve_mask = 0;
    } else if (self->config->filter.single_filter) {
        addr = ( ( (addr & 0x7FF) << 5 ) | (rtr ? 0x10 : 0) );
        mask = ( (mask & 0x7FF) << 5 );
        mask |= 0xFFFFF000;
        preserve_mask = 0;
    } else {
        addr = ( ( (addr & 0x7FF) << 5 ) | (rtr ? 0x10 : 0) );
        mask = ( (mask & 0x7FF) << 5 );
        preserve_mask = 0xFFFF << ( bank == 0 ? 16 : 0 );
        if ( (self->config->filter.acceptance_mask & preserve_mask) == ( 0xFFFF << (bank == 0 ? 16 : 0) ) ) {
            // Other filter accepts all; it will replaced duplicating current filter
            addr = addr | (addr << 16);
            mask = mask | (mask << 16);
            preserve_mask = 0;
        } else {
            addr = addr << (bank == 1 ? 16 : 0);
            mask = mask << (bank == 1 ? 16 : 0);
        }
    }
    self->config->filter.acceptance_code &= preserve_mask;
    self->config->filter.acceptance_code |= addr;
    self->config->filter.acceptance_mask &= preserve_mask;
    self->config->filter.acceptance_mask |= mask;
}

STATIC mp_obj_t machine_hw_can_restart(mp_obj_t self_in) {
    ESP_CHECK_ERROR(twai_initiate_recovery());
    mp_hal_delay_ms(200);
    ESP_CHECK_ERROR(twai_start());
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_hw_can_restart_obj, machine_hw_can_restart);

// TODO: rename
// any() - return `True` if any message waiting, else `False`
STATIC mp_obj_t machine_hw_can_any(mp_obj_t self_in) {
    twai_status_info_t status = _machine_hw_can_get_status();
    return mp_obj_new_bool((status.msgs_to_rx) > 0);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_hw_can_any_obj, machine_hw_can_any);


// Get the state of the controller
STATIC mp_obj_t machine_hw_can_state(mp_obj_t self_in) {
    twai_status_info_t status = _machine_hw_can_get_status();
    return mp_obj_new_int(status.state);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_hw_can_state_obj, machine_hw_can_state);

// Get info about error states and TX/RX buffers
STATIC mp_obj_t machine_hw_can_info(size_t n_args, const mp_obj_t *args) {
    machine_can_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    mp_obj_list_t *list;
    if (n_args == 1) {
        list = MP_OBJ_TO_PTR(mp_obj_new_list(8, NULL));
    } else {
        if (!mp_obj_is_type(args[1], &mp_type_list)) {
            mp_raise_TypeError(NULL);
        }
        list = MP_OBJ_TO_PTR(args[1]);
        if (list->len < 8) {
            mp_raise_ValueError(NULL);
        }
    }
    twai_status_info_t status = _machine_hw_can_get_status();
    list->items[0] = MP_OBJ_NEW_SMALL_INT(status.tx_error_counter);
    list->items[1] = MP_OBJ_NEW_SMALL_INT(status.rx_error_counter);
    list->items[2] = MP_OBJ_NEW_SMALL_INT(self->num_error_warning);
    list->items[3] = MP_OBJ_NEW_SMALL_INT(self->num_error_passive);
    list->items[4] = MP_OBJ_NEW_SMALL_INT(self->num_bus_off);
    list->items[5] = MP_OBJ_NEW_SMALL_INT(status.msgs_to_tx);
    list->items[6] = MP_OBJ_NEW_SMALL_INT(status.msgs_to_rx);
    list->items[7] = mp_const_none;
    return MP_OBJ_FROM_PTR(list);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_VAR_BETWEEN(machine_hw_can_info_obj, 1, 2, machine_hw_can_info);

// Get Alert info
STATIC mp_obj_t machine_hw_can_alert(mp_obj_t self_in) {
    uint32_t alerts;
    uint32_t status = twai_read_alerts(&alerts, 0);
    if (status != ESP_OK) {
        mp_raise_OSError(-status);
    }
    return mp_obj_new_int(alerts);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_hw_can_alert_obj, machine_hw_can_alert);

// Clear TX Queue
STATIC mp_obj_t machine_hw_can_clear_tx_queue(mp_obj_t self_in) {
    return mp_obj_new_bool(twai_clear_transmit_queue() == ESP_OK);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_hw_can_clear_tx_queue_obj, machine_hw_can_clear_tx_queue);

// Clear RX Queue
STATIC mp_obj_t machine_hw_can_clear_rx_queue(mp_obj_t self_in) {
    return mp_obj_new_bool(twai_clear_receive_queue() == ESP_OK);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_hw_can_clear_rx_queue_obj, machine_hw_can_clear_rx_queue);

// send([data], id, *)
STATIC mp_obj_t machine_hw_can_send(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum {
        ARG_data,
        ARG_id,
        ARG_timeout,
        ARG_rtr,
        ARG_self
    };
    static const mp_arg_t allowed_args[] = {
        {MP_QSTR_data, MP_ARG_REQUIRED | MP_ARG_OBJ, {.u_obj = MP_OBJ_NULL}},
        {MP_QSTR_id, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = 0}},
        {MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0}},
        {MP_QSTR_rtr, MP_ARG_KW_ONLY | MP_ARG_BOOL, {.u_bool = false}},
    };

    // parse args
    machine_can_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    // populate message
    size_t length;
    mp_obj_t *items;
    mp_obj_get_array(args[ARG_data].u_obj, &length, &items);
    if (length > 8) {
        mp_raise_ValueError("CAN data field too long");
    }
    uint8_t flags = (args[ARG_rtr].u_bool ? TWAI_MSG_FLAG_RTR : TWAI_MSG_FLAG_NONE);
    uint32_t id = args[ARG_id].u_int;
    if (self->extframe) {
        flags += TWAI_MSG_FLAG_EXTD;
        id &= 0x1FFFFFFF;
    } else {
        id &= 0x7FF;
    }
    if (self->loopback) {
        flags += TWAI_MSG_FLAG_SELF;
    }
    twai_message_t tx_msg = { .data_length_code = length,
                             .identifier = id,
                             .flags = flags
                           };
    for (uint8_t i = 0; i < length; i++) {
        tx_msg.data[i] = mp_obj_get_int(items[i]);
    }
    if (_machine_hw_can_get_status().state == TWAI_STATE_RUNNING) {
        int status = twai_transmit(&tx_msg, args[ARG_timeout].u_int);
        if (status != ESP_OK) {
            mp_raise_OSError(-status);
        }
        return mp_const_none;
    } else {
        nlr_raise(mp_obj_new_exception_msg(&mp_type_RuntimeError, "CAN Device is not ready"));
    }
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_hw_can_send_obj, 3, machine_hw_can_send);

// recv(list=None, *, timeout=5000)
STATIC mp_obj_t machine_hw_can_recv(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum {
        ARG_list,
        ARG_timeout
    };
    static const mp_arg_t allowed_args[] = {
//        { MP_QSTR_list, MP_ARG_OBJ, {.u_rom_obj = MP_ROM_NONE} },
        { MP_QSTR_list, MP_ARG_OBJ, {.u_obj = mp_const_none} },

        { MP_QSTR_timeout, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 5000} },
    };

    // parse args
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    twai_message_t rx_message;
    int status = twai_receive(&rx_message, args[ARG_timeout].u_int);
    if (status != ESP_OK) {
        mp_raise_OSError(-status);
    }
    // Create the tuple, or get the list, that will hold the return values
    // Also populate the fourth element, either a new bytes or reuse existing memoryview
    mp_obj_t ret_obj = args[ARG_list].u_obj;
    mp_obj_t *items;
    if (ret_obj == mp_const_none) {
        ret_obj = mp_obj_new_tuple(4, NULL);
        items = ( (mp_obj_tuple_t *)MP_OBJ_TO_PTR(ret_obj) )->items;
        items[3] = mp_obj_new_bytes(rx_message.data, rx_message.data_length_code);
    } else {
        // User should provide a list of length at least 4 to hold the values
        if (!mp_obj_is_type(ret_obj, &mp_type_list)) {
            mp_raise_TypeError(NULL);
        }
        mp_obj_list_t *list = MP_OBJ_TO_PTR(ret_obj);
        if (list->len < 4) {
            mp_raise_ValueError(NULL);
        }
        items = list->items;
        // Fourth element must be a memoryview which we assume points to a
        // byte-like array which is large enough, and then we resize it inplace
        if (!mp_obj_is_type(items[3], &mp_type_memoryview)) {
            mp_raise_TypeError(NULL);
        }
        mp_obj_array_t *mv = MP_OBJ_TO_PTR(items[3]);
        if (!(mv->typecode == (MP_OBJ_ARRAY_TYPECODE_FLAG_RW | BYTEARRAY_TYPECODE) || (mv->typecode | 0x20) == (MP_OBJ_ARRAY_TYPECODE_FLAG_RW | 'b'))) {
            mp_raise_ValueError(NULL);
        }
        mv->len = rx_message.data_length_code;
        memcpy(mv->items, rx_message.data, rx_message.data_length_code);
    }
    items[0] = MP_OBJ_NEW_SMALL_INT(rx_message.identifier);
    items[1] = mp_obj_new_bool(rx_message.flags && TWAI_MSG_FLAG_RTR > 0);
    items[2] = 0;
    return ret_obj;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_hw_can_recv_obj, 0, machine_hw_can_recv);

// Clear filters setting
STATIC mp_obj_t machine_hw_can_clearfilter(mp_obj_t self_in) {
    machine_can_obj_t *self = MP_OBJ_TO_PTR(self_in);
    self->config->filter.single_filter = self->extframe;
    self->config->filter.acceptance_code = 0;
    self->config->filter.acceptance_mask = 0xFFFFFFFF;
    ESP_CHECK_ERROR(twai_stop());
    ESP_CHECK_ERROR(twai_driver_uninstall());
    ESP_CHECK_ERROR(twai_driver_install(
                         &self->config->general,
                         &self->config->timing,
                         &self->config->filter));
    ESP_CHECK_ERROR(twai_start());
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_hw_can_clearfilter_obj, machine_hw_can_clearfilter);

// bank: 0 or 1(only for std)
// mode: FILTER_RAW_SINGLE, FILTER_RAW_DUAL or FILTER_ADDR_SINGLE or FILTER_ADDR_DUAL
// params: [id, mask]
// rtr: ignored if FILTER_RAW
// Set CAN HW filter
STATIC mp_obj_t machine_hw_can_setfilter(size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum
    {
        ARG_bank,
        ARG_mode,
        ARG_params,
        ARG_rtr
    };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_bank, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_mode, MP_ARG_REQUIRED | MP_ARG_INT },
        { MP_QSTR_params, MP_ARG_REQUIRED | MP_ARG_OBJ },
        { MP_QSTR_rtr, MP_ARG_KW_ONLY | MP_ARG_OBJ, {.u_bool = false} },
    };

    // parse args
    machine_can_obj_t *self = MP_OBJ_TO_PTR(pos_args[0]);
    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args - 1, pos_args + 1, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);

    size_t len;
    mp_obj_t *params;
    mp_obj_get_array(args[ARG_params].u_obj, &len, &params);
    if (len != 2) {
        mp_raise_ValueError("params shall be a 2-values list");
    }
    uint32_t id = mp_obj_get_int(params[0]);
    uint32_t mask = mp_obj_get_int(params[1]); // FIXME: Overflow in case 0xFFFFFFFF for mask
    if (args[ARG_mode].u_int == FILTER_RAW_SINGLE || args[ARG_mode].u_int == FILTER_RAW_DUAL) {
        self->config->filter.single_filter = (args[ARG_mode].u_int == FILTER_RAW_SINGLE);
        self->config->filter.acceptance_code = id;
        self->config->filter.acceptance_mask = mask;
    } else {
        self->config->filter.single_filter = self->extframe;
        _machine_hw_can_set_filter(self, id, mask, args[ARG_bank].u_int, args[ARG_rtr].u_int);
    }
    ESP_CHECK_ERROR(twai_stop() );
    ESP_CHECK_ERROR(twai_driver_uninstall() );
    ESP_CHECK_ERROR(twai_driver_install(
                          &self->config->general,
                          &self->config->timing,
                          &self->config->filter) );
    ESP_CHECK_ERROR(twai_start() );
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_hw_can_setfilter_obj, 1, machine_hw_can_setfilter);

STATIC void machine_hw_can_print(const mp_print_t *print, mp_obj_t self_in, mp_print_kind_t kind) {
    machine_can_obj_t *self = MP_OBJ_TO_PTR(self_in);
    if (self->config->initialized) {
        qstr mode;
        switch (self->config->general.mode)
        {
        case TWAI_MODE_LISTEN_ONLY:
            mode = MP_QSTR_LISTEN;
            break;
        case TWAI_MODE_NO_ACK:
            mode = MP_QSTR_NO_ACK;
            break;
        case TWAI_MODE_NORMAL:
            mode = MP_QSTR_NORMAL;
            break;
        default:
            mode = MP_QSTR_UNKNOWN;
            break;
        }
        mp_printf(print,
                  "CAN(tx=%u, rx=%u, baudrate=%ukb, mode=%q, loopback=%u, extframe=%u)",
                  self->config->general.tx_io,
                  self->config->general.rx_io,
                  self->config->baudrate,
                  mode,
                  self->loopback,
                  self->extframe);
    } else {
        mp_printf(print, "CAN Device is not initialized");
    }
}


static void make_received_message(machine_can_obj_t *self) {
    mp_obj_t ret_obj = mp_obj_new_tuple(4, NULL);
    mp_obj_t *items = ( (mp_obj_tuple_t *)MP_OBJ_TO_PTR(ret_obj) )->items;
    uint8_t data[8];
    items[3] = mp_obj_new_bytes(data, 8);
    self->received_object = ret_obj;
    self->received_object_items = items;
}

static void receiving_task(void *self_in) {
    ESP_LOGE(DEVICE_NAME, "receiver started with %p", self_in);

    machine_can_obj_t *self = self_in;

    for (;;) {
        twai_message_t rx_message;
        int status = twai_receive(&rx_message, 1000);
        ESP_LOGE(DEVICE_NAME, "receive: %d", status);
        if (status != ESP_OK) {
            continue;
        }
        if (self->receiver_task == 0 || self->receiver_task == mp_const_none) {
            ESP_LOGE(DEVICE_NAME, "no callback set, ignoring; status=%d", status);
            continue;
        }
        mp_obj_t *items = self->received_object_items;
        ESP_LOGE(DEVICE_NAME, "obj=%p, items=%p, i3=%p%p", self->received_object, items, items[3], MP_OBJ_TO_PTR(items[3]));
        items[0] = MP_OBJ_NEW_SMALL_INT(rx_message.identifier);
        items[1] = mp_obj_new_bool(rx_message.flags && TWAI_MSG_FLAG_RTR > 0);
        items[2] = 0;
        mp_obj_array_t *mv = MP_OBJ_TO_PTR(items[3]);
        if (!(mv->typecode == (MP_OBJ_ARRAY_TYPECODE_FLAG_RW | BYTEARRAY_TYPECODE) || (mv->typecode | 0x20) == (MP_OBJ_ARRAY_TYPECODE_FLAG_RW | 'b'))) {
            ESP_LOGE(DEVICE_NAME, "Bad item #3 in canarg, tc = 0x%02x %d", mv->typecode, mv->typecode);
            continue;
        }
        mv->len = rx_message.data_length_code;
        memcpy(mv->items, rx_message.data, rx_message.data_length_code);
        ESP_LOGE(DEVICE_NAME, "starting schedule");
        mp_sched_schedule(self->receiver_task, self->received_object);
        ESP_LOGE(DEVICE_NAME, "done");
    }
}



// static mp_obj_t _receive_callback;
//
// static void trigger_mp_schedule() {
//     if (_receive_callback == 0) return;
//     if (_receive_callback == mp_const_none) return;
//     mp_sched_schedule(_receive_callback, mp_const_none);
//     mp_hal_wake_main_task_from_isr();
// }

// NOTE: you must define this in twai.c
// Add to the end of twai_handle_rx_buffer_frames:
//     if (twai_receive_isr_hook != 0) (*twai_receive_isr_hook)();


STATIC mp_obj_t machine_hw_can_set_callback(mp_obj_t self_in, mp_obj_t value_in) {
    machine_can_obj_t *self = self_in;
    if (self->receiver_task != 0) {
        ESP_LOGE(DEVICE_NAME, "Receiving task for %p already created at %p, ignoring", self, self->receiver_task);
    } else {
        const unsigned short stack_depth_in_words = 3 * configMINIMAL_STACK_SIZE;
        int status = xTaskCreate(receiving_task,
                                "CAN receiver",
                                stack_depth_in_words,
                                self, // params
                                0, //   UBaseType_t uxPriority,
                                &self->receiver_task);

        ESP_LOGE(DEVICE_NAME, "Created Task %p for %p", self->receiver_task, self);
        if (status != pdPASS) {
            ESP_LOGE(DEVICE_NAME, "Cannot create task: %p", self);
        }
    }
    make_received_message(self);
    self->rxcallback = value_in;
    ESP_LOGE(DEVICE_NAME, "Callback set to %p", value_in);
    return mp_const_none;
}
MP_DEFINE_CONST_FUN_OBJ_2(can_set_callback_obj, machine_hw_can_set_callback);


// init(tx, rx, baudrate, mode = TWAI_MODE_NORMAL, tx_queue = 2, rx_queue = 5)
STATIC mp_obj_t machine_hw_can_init(size_t n_args, const mp_obj_t *args, mp_map_t *kw_args) {
    machine_can_obj_t *self = MP_OBJ_TO_PTR(args[0]);
    if (self->config->initialized) {
        ESP_LOGW(DEVICE_NAME, "Device is already initialized");
        return mp_const_none;
    }
    return machine_hw_can_init_helper(self, n_args - 1, args + 1, kw_args);
}
STATIC MP_DEFINE_CONST_FUN_OBJ_KW(machine_hw_can_init_obj, 4, machine_hw_can_init);

// deinit()
STATIC mp_obj_t machine_hw_can_deinit(const mp_obj_t self_in) {
    const machine_can_obj_t *self = &machine_can_obj;
    if (self->config->initialized != true) {
        ESP_LOGW(DEVICE_NAME, "Device is not initialized");
        return mp_const_none;
    }
    ESP_CHECK_ERROR(twai_stop());
    ESP_CHECK_ERROR(twai_driver_uninstall());
    self->config->initialized = false;
    return mp_const_none;
}
STATIC MP_DEFINE_CONST_FUN_OBJ_1(machine_hw_can_deinit_obj, machine_hw_can_deinit);

// CAN(bus, ...) No argument to get the object
// If no arguments are provided, the initialized object will be returned
mp_obj_t machine_hw_can_make_new(const mp_obj_type_t *type, size_t n_args,
                                 size_t n_kw, const mp_obj_t *args) {
    // check arguments
    mp_arg_check_num(n_args, n_kw, 1, MP_OBJ_FUN_ARGS_MAX, true);
    if (mp_obj_is_int(args[0]) != true) {
        mp_raise_TypeError("bus must be a number");
    }
    mp_uint_t can_idx = mp_obj_get_int(args[0]);
    if (can_idx > 1) {
        nlr_raise(mp_obj_new_exception_msg_varg(&mp_type_ValueError, "CAN(%d) doesn't exist", can_idx));
    }
    machine_can_obj_t *self = &machine_can_obj;
    ESP_LOGE(DEVICE_NAME, "Calling CAN.new with self=%p", self);

    if (n_args > 1 || n_kw > 0) {
        if (self->config->initialized) {
            // The caller is requesting a reconfiguration of the hardware
            // this can only be done if the hardware is in init mode
            ESP_LOGW(DEVICE_NAME, "Device is going to be reconfigured");
            machine_hw_can_deinit(&self);
        }
        self->rxcallback = mp_const_none;
        self->rx_state = RX_STATE_FIFO_EMPTY;

        // start the peripheral
        mp_map_t kw_args;
        mp_map_init_fixed_table(&kw_args, n_kw, args + n_args);
        machine_hw_can_init_helper(self, n_args - 1, args + 1, &kw_args);
    }
    return MP_OBJ_FROM_PTR(self);
}



// init(mode, extframe=False, baudrate=500, *)
STATIC mp_obj_t machine_hw_can_init_helper(machine_can_obj_t *self, size_t n_args, const mp_obj_t *pos_args, mp_map_t *kw_args) {
    enum {
        ARG_mode,
        ARG_extframe,
        ARG_baudrate,
        ARG_prescaler,
        ARG_sjw,
        ARG_bs1,
        ARG_bs2,
        ARG_tx_io,
        ARG_rx_io,
        ARG_tx_queue,
        ARG_rx_queue,
        ARG_auto_restart
    };
    static const mp_arg_t allowed_args[] = {
        { MP_QSTR_mode, MP_ARG_REQUIRED | MP_ARG_INT, {.u_int = TWAI_MODE_NORMAL} },
        { MP_QSTR_extframe, MP_ARG_BOOL, {.u_bool = false} },
        { MP_QSTR_baudrate, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_prescaler, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = CAN_DEFAULT_PRESCALER} },
        { MP_QSTR_sjw, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = CAN_DEFAULT_SJW} },
        { MP_QSTR_bs1, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = CAN_DEFAULT_BS1} },
        { MP_QSTR_bs2, MP_ARG_KW_ONLY | MP_ARG_INT, {.u_int = CAN_DEFAULT_BS2} },
        { MP_QSTR_tx_io, MP_ARG_INT, {.u_int = 4} },
        { MP_QSTR_rx_io, MP_ARG_INT, {.u_int = 2} },
        { MP_QSTR_tx_queue, MP_ARG_INT, {.u_int = 0} },
        { MP_QSTR_rx_queue, MP_ARG_INT, {.u_int = 5} },
        { MP_QSTR_auto_restart, MP_ARG_BOOL, {.u_bool = false} },
    };
    self->rxcallback = mp_const_none; // disable callback

    mp_arg_val_t args[MP_ARRAY_SIZE(allowed_args)];
    mp_arg_parse_all(n_args, pos_args, kw_args, MP_ARRAY_SIZE(allowed_args), allowed_args, args);
    // Configure device
    self->config->general.mode = args[ARG_mode].u_int & 0x0F;
    self->config->general.tx_io = args[ARG_tx_io].u_int;
    self->config->general.rx_io = args[ARG_rx_io].u_int;
    self->config->general.clkout_io = TWAI_IO_UNUSED;
    self->config->general.bus_off_io = TWAI_IO_UNUSED;
    self->config->general.tx_queue_len = args[ARG_tx_queue].u_int;
    self->config->general.rx_queue_len = args[ARG_rx_queue].u_int;
    self->config->general.alerts_enabled = TWAI_ALERT_AND_LOG || TWAI_ALERT_BELOW_ERR_WARN || TWAI_ALERT_ERR_ACTIVE || TWAI_ALERT_BUS_RECOVERED ||
                                           TWAI_ALERT_ABOVE_ERR_WARN || TWAI_ALERT_BUS_ERROR || TWAI_ALERT_ERR_PASS || TWAI_ALERT_BUS_OFF;
    self->config->general.clkout_divider = 0;
    self->loopback = ((args[ARG_mode].u_int & 0x10) > 0);
    self->extframe = args[ARG_extframe].u_bool;
    if (args[ARG_auto_restart].u_bool) {
        mp_raise_NotImplementedError("Auto-restart not supported");
    }
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    self->config->filter.single_filter = self->extframe;
    self->config->filter.acceptance_code = f_config.acceptance_code;
    self->config->filter.acceptance_mask = f_config.acceptance_mask;
    twai_timing_config_t *timing;
    switch ((int)args[ARG_baudrate].u_int) {
    case 0:
        // specify register values. Your are on your own ...
        timing = &((twai_timing_config_t) {
            .brp = args[ARG_prescaler].u_int,
            .sjw = args[ARG_sjw].u_int,
            .tseg_1 = args[ARG_bs1].u_int,
            .tseg_2 = args[ARG_bs1].u_int,
            .triple_sampling = false
        });
        break;
    case 25:
        timing = &( (twai_timing_config_t) TWAI_TIMING_CONFIG_25KBITS() );
        break;
    case 50:
        timing = &( (twai_timing_config_t) TWAI_TIMING_CONFIG_50KBITS() );
        break;
    case 100:
        timing = &( (twai_timing_config_t) TWAI_TIMING_CONFIG_100KBITS() );
        break;
    case 125:
        timing = &( (twai_timing_config_t) TWAI_TIMING_CONFIG_125KBITS() );
        break;
    case 250:
        timing = &( (twai_timing_config_t) TWAI_TIMING_CONFIG_250KBITS() );
        break;
    case 500:
        timing = &( (twai_timing_config_t) TWAI_TIMING_CONFIG_500KBITS() );
        break;
    case 800:
        timing = &( (twai_timing_config_t) TWAI_TIMING_CONFIG_800KBITS() );
        break;
    case 1000:
        timing = &( (twai_timing_config_t) TWAI_TIMING_CONFIG_1MBITS() );
        break;
    default:
        mp_raise_ValueError("Unable to set baudrate, use one of 25, 50, 100, 125, 250, 500, 800, 1000");
        self->config->baudrate = 0;
        return mp_const_none;
    }
    self->config->timing = *timing;
    self->config->baudrate = args[ARG_baudrate].u_int;

    uint32_t status = twai_driver_install(
                          &self->config->general,
                          &self->config->timing,
                          &(twai_filter_config_t) TWAI_FILTER_CONFIG_ACCEPT_ALL() );
    if (status != ESP_OK) {
        mp_raise_OSError(-status);
    } else {
        status = twai_start();
        if (status != ESP_OK) {
            mp_raise_OSError(-status);
        } else {
            self->config->initialized = true;
        }
    }
    return mp_const_none;
}



STATIC const mp_rom_map_elem_t machine_can_locals_dict_table[] = {
    // CAN_ATTRIBUTES
    { MP_ROM_QSTR(MP_QSTR___name__), MP_ROM_QSTR(MP_QSTR_CAN) },
    // Micropython Generic API
    { MP_ROM_QSTR(MP_QSTR_callback), MP_ROM_PTR(&can_set_callback_obj) },

    { MP_ROM_QSTR(MP_QSTR_init), MP_ROM_PTR(&machine_hw_can_init_obj) },
    { MP_ROM_QSTR(MP_QSTR_deinit), MP_ROM_PTR(&machine_hw_can_deinit_obj) },
    { MP_ROM_QSTR(MP_QSTR_restart), MP_ROM_PTR(&machine_hw_can_restart_obj) },
    { MP_ROM_QSTR(MP_QSTR_state), MP_ROM_PTR(&machine_hw_can_state_obj) },
    { MP_ROM_QSTR(MP_QSTR_info), MP_ROM_PTR(&machine_hw_can_info_obj) },
    { MP_ROM_QSTR(MP_QSTR_any), MP_ROM_PTR(&machine_hw_can_any_obj) },
    { MP_ROM_QSTR(MP_QSTR_send), MP_ROM_PTR(&machine_hw_can_send_obj) },
    { MP_ROM_QSTR(MP_QSTR_recv), MP_ROM_PTR(&machine_hw_can_recv_obj) },
    { MP_ROM_QSTR(MP_QSTR_setfilter), MP_ROM_PTR(&machine_hw_can_setfilter_obj) },
    { MP_ROM_QSTR(MP_QSTR_clearfilter), MP_ROM_PTR(&machine_hw_can_clearfilter_obj) },
    // ESP32 Specific API
    { MP_OBJ_NEW_QSTR(MP_QSTR_clear_tx_queue), MP_ROM_PTR(&machine_hw_can_clear_tx_queue_obj) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_clear_rx_queue), MP_ROM_PTR(&machine_hw_can_clear_rx_queue_obj) },
    { MP_OBJ_NEW_QSTR(MP_QSTR_get_alerts), MP_ROM_PTR(&machine_hw_can_alert_obj) },
    // CAN_MODE
    { MP_ROM_QSTR(MP_QSTR_NORMAL), MP_ROM_INT(TWAI_MODE_NORMAL) },
    { MP_ROM_QSTR(MP_QSTR_LOOPBACK), MP_ROM_INT(TWAI_MODE_NORMAL | 0x10) },
    { MP_ROM_QSTR(MP_QSTR_SILENT), MP_ROM_INT(TWAI_MODE_NO_ACK) },
    { MP_ROM_QSTR(MP_QSTR_SILENT_LOOPBACK), MP_ROM_INT(TWAI_MODE_NO_ACK | 0x10) },
    { MP_ROM_QSTR(MP_QSTR_LISTEN_ONLY), MP_ROM_INT(TWAI_MODE_LISTEN_ONLY) },
    // CAN_STATE
    { MP_ROM_QSTR(MP_QSTR_STOPPED), MP_ROM_INT(TWAI_STATE_STOPPED) },
    { MP_ROM_QSTR(MP_QSTR_ERROR_ACTIVE), MP_ROM_INT(TWAI_STATE_RUNNING) },
    { MP_ROM_QSTR(MP_QSTR_BUS_OFF), MP_ROM_INT(TWAI_STATE_BUS_OFF) },
    { MP_ROM_QSTR(MP_QSTR_RECOVERING), MP_ROM_INT(TWAI_STATE_RECOVERING) },
    // CAN_FILTER_MODE
    { MP_ROM_QSTR(MP_QSTR_FILTER_RAW_SINGLE), MP_ROM_INT(FILTER_RAW_SINGLE) },
    { MP_ROM_QSTR(MP_QSTR_FILTER_RAW_DUAL), MP_ROM_INT(FILTER_RAW_DUAL) },
    { MP_ROM_QSTR(MP_QSTR_FILTER_ADDRESS), MP_ROM_INT(FILTER_ADDRESS) },
};
STATIC MP_DEFINE_CONST_DICT(machine_can_locals_dict, machine_can_locals_dict_table);



// singleton CAN device object
machine_can_config_t can_config = { .general = TWAI_GENERAL_CONFIG_DEFAULT(2, 4, 0),
                                    .filter = TWAI_FILTER_CONFIG_ACCEPT_ALL(),
                                    .timing = TWAI_TIMING_CONFIG_25KBITS(),
                                    .initialized = false
                                  };

                                  // Python object definition
const mp_obj_type_t machine_can_type = {
    {&mp_type_type},
    .name = MP_QSTR_CAN,
    .print = machine_hw_can_print,
    .make_new = machine_hw_can_make_new,
    .locals_dict = (mp_obj_dict_t *)&machine_can_locals_dict,
};

STATIC machine_can_obj_t machine_can_obj = { {&machine_can_type}, .config = &can_config, .receiver_task = 0 };


// #endif // MICROPY_HW_ENABLE_CAN
