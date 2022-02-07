Step1: 
	Step1-1: open the Files app to "~/svlsimulator-linux64-2021.2.2" folder, and open the "simulator".
	Step1-2: ...
	...
	Step1-...: enter Simulator app
Step2:
	Step2-1: open the Terminal app, and cd into "~/adehome/AutowareAuto".
	Step2-2: type in "source .aderc-amd64-foxy-lgsvl-nvidia"
	Step2-3: type in "ade start --enter". 
	Step2-4: type in "nvidia-smi" to check if gpu is working or not.
	Step2-5: (if somethings goes wrong or want to quit, type in "exit", and then type in "ade stop".)
Step3:
	Step3-1: cd into "~/AutowareAuto".
	Step3-2: type in "source install/setup.bash".
	Step3-3: type in "lgsvl_bridge".
	Step3-4: switch to Simulator app, and press the Play button on the buttom left.
	Step3-5: open another Terminal app, and type in "ade enter".
	Step3-6: Should see the sensors' information being updated in both Terminal and Simulator app now.
Step4:
	Use right mouse key to drag the view angle of the car in the Simulator app.
