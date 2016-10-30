#include "task.h"

Task::Task (){
	m_pfnTask = 0;
	m_microsPerReading = TASK_DEFAULT_INTERVAL;
	m_microsPrevious = 0;
	m_enabled=false;
}
void Task::initialize (unsigned short taskId, float rate, task_func pfn){

  m_microsPerReading = 1000000/rate;
	
	if(m_microsPerReading < TASK_MIN_INTERVAL || pfn == 0){
		m_enabled = false;

#if defined DEBUG_MODE   
  Serial.print("Task NOT initialized: ");
  Serial.println(taskId);
#endif

		return;
	}

#if defined DEBUG_MODE   
  Serial.print("Task initialized: ");
  Serial.println(taskId);
#endif

  m_taskId = taskId;
	m_pfnTask = pfn;
	m_enabled = true;
  m_rate = rate;
}

void Task::trigger (unsigned long microsNow)
{
	if(!m_enabled){
		return;
	}
	if((microsNow - m_microsPrevious) > m_microsPerReading){

    m_rate = 1000000.0f/(float)(microsNow - m_microsPrevious);

    #if defined DEBUG_MODE   
      Serial.print(m_taskId);
      Serial.print(" - Rate: ");
      Serial.print(m_rate);
      Serial.print(" Hz - ");
      Serial.print((microsNow - m_microsPrevious));
      Serial.print("/");
      Serial.println(m_microsPerReading);
    #endif
    
		if(m_pfnTask){
			(*m_pfnTask)();
		}
		m_microsPrevious = microsNow;
	}
}
bool Task::isEnabled ()
{
	return m_enabled;
}
void Task::setEnabled (bool b)
{
	m_enabled = b;
}
float Task::getRate () {
  return m_rate;
}






