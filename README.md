# Robot-code-2025-primo

hello dear members. for some of you this is going to be you're first code in first.
to save me and you some time here is a part docs about...

# how to write a good and orgenize code docs.
remember that you can always check the codes before, and the the PowerPoint presentation.
(TakeFeedSubsystem is just an example, this need to be adjusted to you're code.)
# Structure    
  1. class variable fields
![image](https://github.com/user-attachments/assets/f1cc25c7-ef6a-4d9f-84fd-1953aab39079)
  2. singelton
![image](https://github.com/user-attachments/assets/887cd142-e0a6-4ac0-a5f3-0eb8af915e5b)
  3. constructor - this func needs to be private!!! 
![image](https://github.com/user-attachments/assets/28df7ca6-497b-4b6a-925f-bd9a64b774b4)
  4. runnable commands - do not forget java docs and comments.
  command docs - https://docs.wpilib.org/he/stable/docs/software/commandbased/commands.html
  i will add for each subsystem wich control system and profile (motion magic) you'll need to use. 
![image](https://github.com/user-attachments/assets/842af8af-1284-487f-85de-1c9b4be87084)
  5. configs - at the moment just write at the end of the code a void func name config. 
  6. constance - this need to be written in constance.java and creat a class in there for you're subsystem.
![image](https://github.com/user-attachments/assets/5f019f1a-9da0-48de-9990-91d03c8ea541)

# Conventions 
  statistically this is the number one reason for commits. 
  https://docs.google.com/presentation/d/1MV573jy07D3SsIuoDuUabkCaEArD_9D1/edit#slide=id.p1
# SOLID 
  this part is a suggestion, i do think for you to understand why the code is like it is you must understand this. 
  https://www.digitalocean.com/community/conceptual-articles/s-o-l-i-d-the-first-five-principles-of-object-oriented-design

# Canon - uri
The canon is base on one TalonFX and one Digital switch.
 
*control System* - a normal set func and a position control to adjust the positon of the coral relativ to the canon.
what you'll need to figure out is... "do i need to use a profile? and if i do which one"
https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/device-specific/talonfx/basic-pid-control.html + the next page 

The command are as following.

-collectUntilCoralCommand- straightforward

-collectWhilePressdCommand- straightforward

-adjustCoralCommand- adjust the positon of the coral relativ to the canon using a position control methood.

-lossenCoralCommand- straightforward

# Gripper - noga
there is one TalonFX
 
*control system* - a normal set() func

 the command are as following.

-collectWhilePressCommand- straightforward

-lossenGripCommand- straightforward

-tossCommand- straightforward

# Elevator - shay
there is two TalonFX, one needs to follow the other there is a func for that, and a digital switch

*control system* - figure it out https://v6.docs.ctr-electronics.com/en/2024/docs/api-reference/device-specific/talonfx/basic-pid-control.html + the next page 

 the command are as following.
-setHome- reset the incoders

-homeCommand- use a switch add activate setHome()

-relocateCommand- takes a given position and moves to that position

# Climb - ziv
![to be continued](https://github.com/user-attachments/assets/7db5fb63-70b3-4646-9a80-58158f3e28ee)



