""" Shows how to access/modify the hardware parameters for the microscope """

import win32com.client

# Start communicate with microscoep
zen = win32com.client.GetActiveObject("Zeiss.Micro.Scripting.ZenWrapperLM")

#Get hardware settings
hw_set = zen.Devices.ReadHardwareSetting()
comp_ids = hw_set.GetAllComponentIds()

# Print all parameters for each hardware component
for comp_id in comp_ids:
    print(comp_id)
    for param in hw_set.GetAllParameterNames(comp_id):
        print('\t',param)

# Get objective position and change to new
obj_pos = hw_set.GetParameter('MTBObjectiveChanger', 'Position')
if obj_pos != "2":
    new_pos = "2"
else:
    new_pos = "1"
hw_set.SetParameter('MTBObjectiveChanger', 'Position', new_pos)
zen.Devices.ApplyHardwareSetting(hw_set)
