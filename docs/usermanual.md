usermanual.md
# User Manual

To use the component, you will need to do the following:
* Obtain the Tracker and Controller serial numbers. You can do that by starting Steam, navigating to Devices -> Manage Trackers and displaying the tracking devices. Their serial numbers should appear, and will have a format of *ABC-1A2B3456*. Store these safely. Make sure the serial numbers correspond to the devices (Tracker and Controller) that you are actually intending to use.


<img src="vive_tracker.jpg" alt="drawing" width="200"/> <img src="htc-vive-controller.jpg" alt="drawing" width="200"/>



* Secure the Tracker at a stationary pose, preferably in the world reference frame. This could be the base of your robot, like in the setup below. 

<img src="experimental-setup.png" alt="drawing" width="500"/>


* Position your HTC Vive bases, so that the Tracker and Controller are both in their field of view, like below.

![Experimental setup](experimental-setup-full.png)


* Run the capture script `track_devices.py` as follows:

```text
export PYTHONPATH=$PYTHONPATH:$(pwd)
python src/track_devices.py --host <Orion Context Broker IP> --port <port> --tracker <Tracker Serial Number> --controller <Controller Serial Number>
```
* To capture some pose data, press and hold the Controller's trigger. This will capture pose data as long as you keep it pressed. Upon release, the script updates the device's context in the Context Broker.


To check that the Context Broker is receiving the context updates, you can enter the following:
`while true; do  curl localhost:1026/v2/entities/HTCViveControllerLHR-1D2D7974 -s -S --header 'Accept: application/json' | python -mjson.tool ; sleep 0.25; done`, replacing `LHR-1D2D7974` with your tracker's serial number.