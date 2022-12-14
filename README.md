# Autoinjector 2.0
![autoinjector_2_system](https://user-images.githubusercontent.com/60007263/207665344-35e8f56a-a40b-4dce-8dff-3479653bf2f0.png)
-------------
Autoinjector 2.0 is an updated iteration of the original Autoinjector system ([GitHub - Python3 branch](https://github.com/bsbrl/autoinjector/tree/Python3), [EMBO Journal Paper](https://www.embopress.org/doi/full/10.15252/embr.201947880)) designed for conducting automated microinjections in embryonic mouse brain tissue and human brain organoids. Autoinjector 2.0 improves upon the original Autoinjector by acheiving greater levels of automation via updated hardware, more sophisticted software control, and an upgraded user experience. The main improvements include:
- Zeiss microscope system with automated hardware features including computer controllable objective changer, optovar changer, reflector changer, focus controller, and stage. In addition, the transmitted light source and epifluorescence LED light source are computer controllable.
- Updated pressure control system with computer controllable option for high pressure unclogging of the microinjection needle.
- 3D microscope-manipulator calibration to enable 3D microinjection targetting without needing to update/revise the calibration when changing focus heights (as was required for the original Autoinjector).
- Neural network detection of microinjection needle tip for automated microscope-manipulator calibration process.
- Neural network detection of tissue and tissue edges for automated target annotation.
- Computer vision annotation tracking for accurate targetting during tissue displacement caused by micronjection.

Autoinjector 2.0 was developed at Human Technopole in Milan Italy from June 2022 to December 2022 for the [Taverna Group](https://humantechnopole.it/en/research-groups/taverna-group/). The goal in developing Autoinjector 2.0 was to further increase microinjection throughput (over the original Autoinjector) and accelerate the study of novel tissue types like human brain organoids.	These goals strive to enable more complex biological experiments requiring large numbers of injected cells in well-developed and nascent biological model systems.

As of December 2022, the Autoinjector 2.0 has been used to conduct microinjection of fluorescent dextran into embryonic mouse telencephalon and human brain organoids grown with Pasca and Lancaster protocols.

-------------

The Autoinjector 2.0 is a robotic, vision-guided platform for conducting automated microinjection along a user-specified trajectory in organotypic slices. Autoinjector 2.0 uses robotic control and computer vision to automate laborious and repetitive tasks of the manual microinjection process. This read me takes you through the system requirements and installation instructions. Consult the documentation for a more comprehensive explanation of the Autoinjector 2.0 system.

1. [System Requirements](https://github.com/obria006/Autoinjector_2.0#system-requirements)
	- [Hardware Requirements](https://github.com/obria006/Autoinjector_2.0#hardware-requirements)
	- [Software Requirements](https://github.com/obria006/Autoinjector_2.0#software-requirements)
2. [Install Instructions](https://github.com/obria006/Autoinjector_2.0#install-instructions)
3. [Running the Application](https://github.com/obria006/Autoinjector_2.0#running-the-application)
4. [Troubleshooting](https://github.com/obria006/Autoinjector_2.0#troubleshooting)
5. [License](https://github.com/obria006/Autoinjector_2.0#license)


## System requirements 

### Hardware Requirements
1. Computer (Windows 10 OS)
	- GPU is not necessary. While a GPU will increase speed of neural network detections, the computer used during development only used the CPU and was sufficiently speedy.
2. Arduino Uno
3. Zeiss Axio Observer 7 microscope with motorized objective changer, optovar changer, reflector changer, focus controller (Z-drive), and stage.
4. Microscope camera (tested with Hamamatsu Orca Flash 4.0)
	- A complete list of available cameras can be found at [Micro-Manager's device support](https://micro-manager.org/Device_Support). Additional cameras may be used (for instance the [Dage-MTI IR-2000](https://dagemti.com/products/cameras/ir-2000-camera/)), but additional custom Python software must be written to support these cameras. 
5. Sensapex uMp-4 4-axis micromanipulator. 
6. Custom pressure rig.
	- [EMBO Autoinjector Journal Paper](https://www.embopress.org/doi/full/10.15252/embr.201947880) for some details on pressure controller. Supplementary information includes info on pressure controller.
	- [Setting up and using the autopatcher for automated intracellular neural recording in vivo - Supplementary Data 4](https://www.ncbi.nlm.nih.gov/pmc/articles/PMC4877510/) for PCB files.

### Software Requirements
Autoinjector 2.0 has only been tested with Windows 10 using Python 3.9.13.
1. [Python 3.9.13 and Autoinjector software](https://github.com/obria006/Autoinjector_2.0#1-python-and-autoinjector-software)
2. [Arduino IDE](https://github.com/obria006/Autoinjector_2.0#2-arduino)
3. [Micromanager 2.0+](https://github.com/obria006/Autoinjector_2.0#3-micromanager)
4. [Zeiss ZEN Pro microscope software](https://github.com/obria006/Autoinjector_2.0#4-zen-interface)
5. [Your camera driver](https://github.com/obria006/Autoinjector_2.0#5-your-camera-driver)


## Install Instructions

Install the following software to operate the Autoinjector 2.0. It is highly recommended to create a virtual environment in which to install the Autoinjector 2.0 software.


### 1. Python and Autoinjector software
1. Download [Python 3.9.13](https://www.python.org/downloads/release/python-3913/) for Windows. For our 64-bit Windows 10 computer we downloaded the *"Windows installer (64-bit)"* file.

2. Launch the installer and follow the prompts to install Python on your computer.

3. (Optional but highly recommended) Create a Python 3.9.13 virtual environment to contain and manage the Autoinjector 2.0 files. A virtual environment is a way to compartmentalize Python projects, so that one project requiring certain software versions doesn't impact a project with different software version requirements. [Guidance for installing packages with pip and virtual environments.](https://packaging.python.org/en/latest/guides/installing-using-pip-and-virtual-environments/)

    - If you are unfamiliar with virtual environments, one method to create an environment is to run the following command in the Windows command prompt: `path-to-python-installation -m -venv path-to-create-env`. See [venv documentation](https://docs.python.org/3/library/venv.html) for more information. A "real-world" example of the command is:
	
    ```
    C:\Program Files\Python39\python.exe -m venv C:\Users\Public\Documents\Autoinjector_2
    ```
	
    - The above command uses the Python installation located at *"C:\Program Files\Python39\python.exe"* to create a virtual environment in a directory located at *"C:\Users\Public\Documents\Autoinjector_2"*.

4. (Required if using a virtual environment) Activate the virtual environment. See ["Activating a virtual environment"](https://packaging.python.org/en/latest/guides/installing-using-pip-and-virtual-environments/#activating-a-virtual-environment) for guidance. After this step, all the following commands must be in the Windows command prompt of an activated virtual environment (if using a virtual environment).
    
    - To activate the virtual environment you must run the "activate" file in the created virtual environment. To activate the virtual environment created above, run the following command in the Windows command prompt:
    ```
    C:\Users\Public\Documents\Autoinjector_2\Scripts\activate
    ```

5. Download the Autoinjector files from the GitHub.

    - (Option 1) Clone the repository to your virtual environment/desired directory.

    - (Option 2) Download a zip file of the code by clicking the green "Code" box in the upper right corner of the GitHub page and selecting "Download ZIP". After its downloaded, extract its contents to your virtual environment/desired directory.

6. Install the necessary dependencies of the Autoinjector project. In the Windows Command Prompt, ensure you are in an activated virtual environment (if you are using one), and make sure you are located in the virtual environment directory.
	- Your command prompt should look something like below. The `(Autoinjector_2)` means you are in an activated virtual environment named "Autoinjector_2". If the virtual environment wasn't activated, then you wouldn't see `(Autoinjector_2)`. The `C:\Users\Public\Documents\Autoinjector_2>` means that you are located at in the directory of *"C:\Users\Public\Documents\Autoinjector_2"*.
	```
	(Autoinjector_2) C:\Users\Public\Documents\Autoinjector_2>
	```

	- Navigate into the downloaded Autoinjectr folder with the command `cd Autoinjector`. Now your command prompt should look like:
	```
	(Autoinjector_2) C:\Users\Public\Documents\Autoinjector_2\Autoinjector>
	```

	- Finally you can install Autoinjector software with the following command. This will install the dependencies specified in the *"setup.py"* file and (more importantly) find the necessary packages within the Autoinjector folder:
	```
	python -m pip install .
	```

	- (Optional) To ensure you have the correct dependency versions run the following command:
	```
	python -m pip install -r requirements.txt
	```


### 2. Arduino
1. Download the arduino IDE windows installer [here](https://www.arduino.cc/en/software). For our 64-bit Windows 10 computer, we downloaded the file "Windows Win 10 and newer, 64 bits".

2. Launch the installer and follow installation instructions on screen.

3. Connect your arduino to your computer via USB.
4. In the downloaded Autoinjector folder, open the following file in the Arduino IDE: *"Autoinjector\src\pressure_control\arduino_pressure_control\arduino_pressure_control.ino"*.

3. Follow the instructions to identify your Arduino COM port and connect to the Arduino device to the Arduino IDE as shown in [this tutorial](https://www.arduino.cc/en/Guide/ArduinoUno#toc5). Take note of which COM port your Arduino is on i.e. COM3.

4. Upload the *"arduino_pressure_control.ino"* file to the Arduino device as shown in [this tutorial](https://www.arduino.cc/en/Guide/ArduinoUno#toc6).


### 3. Micromanager
1. Download the micromanager Windows installer [here](https://micro-manager.org/wiki/Download_Micro-Manager_Latest_Release). We downloaded the Micro-Manager 2.0.0 "Windows 64-bit" file.

2. Launch the installer and follow installation instructions on screen.
	- It is highly recommended that you install the file at *"C:/Program Files/Micro-Manager-2.0"*. You will need to reference the Micro-Manager file later when configuring the Autoinjector 2.0.
	* **Note: Your device interface version must match between your Micro-Manager installation and the installed pymmcore Python package version. Details are located at [pymmcore GitHub](https://github.com/micro-manager/pymmcore#matching-micro-manager-and-pymmcore-versions)**

### 4. Zen interface
**Note: Python interfaces with Zeiss ZEN software to control the automated movements of the microscope. This interface is achieved via the COM interface in Zeiss's [Open Application Development](https://github.com/zeiss-microscopy/OAD). Further info/guidance for the COM interface is available on the [OAD GitHub](https://github.com/zeiss-microscopy/OAD/tree/master/Interfaces/COM_interface)**

1. Ensure Zeiss ZEN Pro is installed on the computer. Likely this is already installed on the computer if you have the Zeiss microscope.

2. Download the *"regScripting_Release.bat"* file from Zeiss's [OAD GitHub](https://github.com/zeiss-microscopy/OAD/tree/master/Interfaces/COM_interface/Sourcecode_COM_Python). The file should be located in the OAD GitHub at *"/OAD/Interfaces/COM_interface/Sourcecode_COM_Python"*

3. Open the *"regScripting_Release.bat"* file in a text editor (like Notepad), and modify the paths of `dll-1` and `dll-2` to match the installation paths on your system.

4. Run *"regScripting_Release.bat"* as admin to make the ZEN commands available to Python. 


### 5. Your Camera Driver
**Note: This step might not be necessary. This step was included in the [Autoinjector 1.0 GitHub - master branch](https://github.com/bsbrl/autoinjector), but I didn't actually follow this steps for Autoinjector 2.0. However, the microscope and camera system that I used to develop Autoinjector 2.0 were configured by a Zeiss technician, so they could have installed the camera driver before I arrived.**

From Autinjector 1.0: 
> Follow the instructions for your camera driver install. In our work we have used the [Hamamatsu Orca Camera](https://www.hamamatsu.com/us/en/product/type/C13440-20CU/index.html) and [Photometrics Cool Snap Dyno PVCam](https://www.photometrics.com/products/ccdcams/coolsnap-dyno.php)

## Running the application
Once you've installed the above software, you are ready to run the Autoinjector 2.0 application. To run the application you must run *"configure_autoinjector.py"* (or *"application_minimal.py"* but this will prompt you to open *"configure_autoinjector.py"* if you haven't completed the configuration yet). Run the *"configure_autoinjector.py"* script with the following command in the command prompt: `path-to-virtual-environment-python path-to-configure_autoinjector.py`. A real world example is shown below:

```
C:/Users/Public/Documents/envs/Autoinjector_2/Scripts/python.exe C:/Users/Public/Documents/envs/Autoinjector_2/Autoinjector/configure_autoinjector.py
```

This command means use the Python installed at *"C:/Users/Public/Documents/envs/Autoinjector_2/Scripts/python.exe"* (which is in the virtual environment where you installed the necessary packages) to run the *"C:/Users/Public/Documents/envs/Autoinjector_2/Autoinjector/configure_autoinjector.py"* file.

The *"configure_autoinjector.py"* will open a user interface where you will need to specify several important parameters that dictate how the Autoinjector system operates (shown below). See the user manual for more information about the configuration app. After you enter the parameters, click "Save Configuration" and "Open Autoinjector". If everything is installed correctly, and you specified the correct parameters in the configuration, then the main Autoinjector 2.0 application will open.

![configuration_app](https://user-images.githubusercontent.com/60007263/207665541-240e6fed-a621-4639-961c-5ac0d71ed19d.png)


Once you saved the configuration, you can directly open the Autoinjector 2.0 app (and bypass the configuration app) by running *"application_minimal.py"*. A real world example of this command is shown below:

```
C:/Users/Public/Documents/envs/Autoinjector_2/Scripts/python.exe C:/Users/Public/Documents/envs/Autoinjector_2/Autoinjector/application_minimal.py
```

## Troubleshooting

### Sensapex Issues
**TLDR: If you have sensapex issues try `pip install sensapex==1.22.7`.**

There are some known issues with some versions of the sensapex Python package (but I haven't encountered issues with version 1.22.7). Start with pip installing the sensapex package as detailed above (this automatically happens during the `python -m pip install .` step in [Python 3.9.13 and Autoinjector software](https://github.com/obria006/Autoinjector_2.0#1-python-and-autoinjector-software) installation). 

1. First issue: pip installing the sensapex package (for package versions<=1.22.6) may result in an incomplete installation (a missing piece of software).

	- Follow the guidance in this [GitHub issue](https://github.com/sensapex/sensapex-py/issues/9) to properly install the Sensapex package. 
		- Long story short, to complete the installation, you must download the 1.022 binaries from [Sensapex](http://dist.sensapex.com/misc/um-sdk/latest/) and place the *"libum.dll"* file in the sensapex package folder containing *"sensapex.py"* (i.e. *"/python-installation-path/Lib/site-packages/sensapex"*)

2. Second issue: pip installing sensapex version 1.22.8 simply wouldn't install. 1.22.8 is the most recent at the time of writing this on December 14, 2022.  So `pip install sensapex==1.22.7` to revert to the previous version.

### Micro-Manager Python Package (pymmcore)
Your device interface version must match between your Micro-Manager installation and the installed pymmcore Python package version. Details are located at [pymmcore GitHub](https://github.com/micro-manager/pymmcore#matching-micro-manager-and-pymmcore-versions).

If you keep getting an error in the user interface that says your getting a camera error, but you know that everything is correct (camera is turned on and connected with computer) it may be a symptom of the version error. I've pinned the pymmcore dependency that worked for me in setup.py to overcome this issue at Human Technopole.

## License
This work is lisenced under the MIT lisence. See LISENCE.txt for additional information.  
