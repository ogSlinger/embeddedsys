# Embedded Systems
## Derek Spaulding
## Southern New Hampshire University
## CS 350 - Emerging Sys Arch & Tech 2024
## October 22, 2024
###### Inside 'bitmanip' locate 'uartecho2.c' for code created.
###### Inside 'cs350project' locate 'gpiointerrupt.c' for code created.  


- ### Summarize the project and what problem it was solving.
  - For 'bitmanip' we were taught how to communicate to the board via UART. We were tasked on having the TI CC3320S board LED power one when typing 'ON' in the console, and 'OFF' when wanting to turn it off using a state machine.
  - For the 'cs350project', we were tasked in prototyping the logic for a thermostat. We utilized the on-board temperature reader via I2C, interrupts to increment and decrement the 'set point', and having the board LED come on when heat was required to meet the set point requirement.
 
    
- ### What did you do particularly well?
  - I would consider for the 'bitmanip' artifact as a great way I utilized setting flags on a byte for the exercise. I used three bits to set as flags for the state machine to understand when to turn the board on and off, and when to reset flags when needed.
  - For the 'cs350project', I feel I did well in correcting a previous mistake and further my understanding of utilizing timers and setting flags to be used with the state machines. I also improved with utilizing enums with state machines to help provide readability, as well as made a task scheduler that modularized tasks to be performed by the state machine given their individual periods.

    
- ### Where could you improve?
  - I would require remediation and iteration with cleaning up how my state machines are created and read. I feel I just scratched the surface and wish to deepen my knowledge on them. I also feel I need to understand more C concepts that optimize performance, such as 'static'.

    
- ### What tools and/or resources are you adding to your support network?
  - One frame of reference I am utilizing to give myself a roadmap on things to research into are the interview questions proposed by the class. I cam across an external site that offers a wide range of interview questions I can learn about in depth as I progress towards my bachelor's.

    
- ### What skills from this project will be particularly transferable to other projects and/or course work?
  - I feel the incorporation of state machines is a very great tool. It includes the necessity to consider computational tasks within time limits. For example, game servers with tick rates I would assume use state machines to handle incoming player actions.

    
- ### How did you make this project maintainable, readable, and adaptable?
  - I utilized enums with what I hope to be readable names, such as 'BS_Inc' for button state increment. All functions were abstracted enough to keep them as core components independent from each other, which should make the code easier to maintain.
