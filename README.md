# Pybullet-Control-trial
studying how to control robot in pybullet :D


IK_PD
with p.setJointMotorControl2 in pybullet, by using PD Control, it is able to control 2 link by calculate Inverse Kinematics manually.
<img width="40%" src="https://user-images.githubusercontent.com/72867850/154547113-3ec3a632-3a7e-48b3-999c-f9b013fcb62f.gif"/>


Jacobian_Implement
without using PD Control function in pybullet, by calculate Jacobian manually, it is also able to control 2 link and baxter robot.
It's almost same between implemented function and manually implemented one.
<img width="40%" src="https://user-images.githubusercontent.com/72867850/154547346-bdb0e3bb-7d44-4cea-87a8-7a6c4b399760.gif"/>
<img width="40%" src="https://user-images.githubusercontent.com/72867850/154547421-23cf7eec-7b2b-4cdc-8572-38db9b337aae.gif"/>


IK_laikago
apply IK and PD function to the four-legged robot laikago
<img width="40%" src="https://user-images.githubusercontent.com/72867850/154547533-58731d42-cee5-422d-810a-3de328194713.gif"/>



PPO_Ant
control environment Ant in mujoco with PPO from baseline.
<img width="40%" src="https://user-images.githubusercontent.com/72867850/154542034-9c243949-3dc0-497e-aedd-f96431387081.gif"/>
<img width="40%" src="https://user-images.githubusercontent.com/72867850/154542242-d3a0e963-37e1-45cf-8fbd-99988876bb35.gif"/>
