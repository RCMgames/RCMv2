#ifndef RCMROS_H
#define RCMROS_H
#include "rcm.h"

// rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_node_t node;

void error_loop()
{
    while (1) {
        digitalWrite(ONBOARD_LED, !digitalRead(ONBOARD_LED));
        delay(50);
    }
}

#define declarePub(name, type) \
    rcl_publisher_t name##Pub; \
    type name##Msg;

#define declareSubAndCallback(name, type)                \
    rcl_subscription_t name##Sub;                        \
    type name##Msg;                                      \
    void name##_subscription_callback(const void* msgin) \
    {                                                    \
        const type* name##Msg = (const type*)msgin;

#define createPublisher(name, type)                                      \
    RCCHECK(rclc_publisher_init_best_effort(                             \
        &name##Pub,                                                      \
        &node,                                                           \
        rosidl_typesupport_c__get_message_type_support_handle__##type(), \
        "/rcm/" #name));

#define addSub(name, type)                                               \
    RCCHECK(rclc_subscription_init_default(                              \
        &name##Sub,                                                      \
        &node,                                                           \
        rosidl_typesupport_c__get_message_type_support_handle__##type(), \
        "/rcm/" #name));                                                 \
    RCCHECK(rclc_executor_add_subscription(&executor, &name##Sub, &name##Msg, &name##_subscription_callback, ON_NEW_DATA));

#define RCCHECK(fn)                    \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
            error_loop();              \
        }                              \
    }
#define RCSOFTCHECK(fn)                \
    {                                  \
        rcl_ret_t temp_rc = fn;        \
        if ((temp_rc != RCL_RET_OK)) { \
            ROSCheckFail = true;       \
        }                              \
    }

#define publish(name) RCSOFTCHECK(rcl_publish(&name##Pub, &name##Msg, NULL))

#define rosSpin(time) rclc_executor_spin_some(&executor, RCL_MS_TO_NS(time))

void ROSWifiSettings();
void ROSbegin();
char* nodeName;
int rosWifiTimeout = 1500;
int numSubscribers = 10;
#include <example_interfaces/msg/bool.h>

declareSubAndCallback(enabled, example_interfaces__msg__Bool);
enabled = enabledMsg->data;
ROSCheckFail = false;
lastEnableSentMillis = millis();
} // end of callback

void setupROS()
{
    ROSWifiSettings();
    allocator = rcl_get_default_allocator();
    RCCHECK(rclc_executor_init(&executor, &support.context, numSubscribers, &allocator));
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
    RCCHECK(rclc_node_init_default(&node, nodeName, "", &support));

    addSub(enabled, example_interfaces__msg__Bool);

    ROSbegin();
}

#endif // RCMROS_H
