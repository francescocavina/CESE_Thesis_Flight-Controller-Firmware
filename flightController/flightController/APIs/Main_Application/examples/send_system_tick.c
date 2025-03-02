uint8_t * loggingStr1 = pvPortMalloc(50 * sizeof(uint8_t));

system_tick = xTaskGetTickCount();
sprintf((char *)loggingStr1, "DEBUG Time: %lu [ms]\n", (system_tick * 1000 / configTICK_RATE_HZ));
xQueueSend(USB_Communication_Debug_Queue_Handle, &loggingStr1, 0);
