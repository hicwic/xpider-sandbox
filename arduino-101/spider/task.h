#ifndef _TASK_H_
#define _TASK_H_

#include "globals.h"
#include <arduino.h>

#define TASK_DEFAULT_INTERVAL 50000 //50ms
#define TASK_MIN_INTERVAL 5000 //5ms

typedef void (*task_func)();
class Task{
public:
	Task();
	void initialize (unsigned short taskId, float rate, task_func pfn);
	void trigger (unsigned long microsNow);
	bool isEnabled ();
	void setEnabled (bool b);
  float getRate ();
protected:
	unsigned long m_microsPerReading;
	unsigned long m_microsPrevious;
  float m_rate;
  unsigned short m_taskId;
	task_func m_pfnTask;
	bool m_enabled;
};

#endif






