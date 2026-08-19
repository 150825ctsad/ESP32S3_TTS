#ifndef __RECORDER_H
#define __RECORDER_H

/* 初始化录音任务（VAD 触发 + Base64 编码 + MQTT 上传） */
void recorder_init(void);

#endif // __RECORDER_H
