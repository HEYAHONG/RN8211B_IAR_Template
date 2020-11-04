#include "FreeRTOS.h"
#include "task.h"
void main_task1(void *arg)
{
 
  while(1)
  {
    vTaskDelay(5);
  }
}

void main_task2(void *arg)
{
 
  while(1)
  {
     vTaskDelay(5);
  }
}

int main()
{
  
  /*创建任务*/
  xTaskCreate( main_task1, "main_task1",128, NULL, 1, NULL );
  xTaskCreate( main_task2, "main_task2",128, NULL, 1, NULL );
  
  /* Start the scheduler so the tasks start executing. */
  vTaskStartScheduler();
  while(1);//死循环，一般不会到这里
}