@ECHO OFF
::This is a simple batch file that runs the ekf_ahrs.py
TITLE EKF AHRS
ECHO Running ekf_ahrs.py...........
call "C:\Users\Sakar\Documents\Arduino\libraries\major_project\python_ahrs\venv\Scripts\activate.bat"
python "C:\Users\Sakar\Documents\Arduino\libraries\major_project\python_ahrs\ekf_ahrs.py"
PAUSE