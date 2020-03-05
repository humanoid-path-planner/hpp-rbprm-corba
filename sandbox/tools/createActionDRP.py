# Create actions YAML file

import yaml
#import anymal_darpa_Asil as drp

def autoBaseFootstep(configs,fullbody,filename):
    initial = """\
steps:

 - step:
    - base_auto:
       height:
       average_linear_velocity: 0.5

 - step:
    - base_auto:
       average_linear_velocity: 0.5
    - footstep:
       name: RF_LEG
       target:
        frame: odom
        position: 
        
 - step:
    - base_auto:
       average_linear_velocity: 0.5

 - step:
    - base_auto:
       average_linear_velocity: 0.5
    - footstep:
       name: LH_LEG
       target:
        frame: odom
        position:

 - step:
    - base_auto:
       average_linear_velocity: 0.5
 - step:
    - base_auto:
       average_linear_velocity: 0.5
    - footstep:
       name: LF_LEG
       target:
        frame: odom
        position:

 - step:
    - base_auto:
       average_linear_velocity: 0.5

 - step:
    - base_auto:
       average_linear_velocity: 0.5
    - footstep:
       name: RH_LEG
       target:
        frame: odom
        position:

 - step:
    - base_auto: 
       average_linear_velocity: 0.5 
    """
    data = yaml.load(initial)

    LF_footPos = fullbody.getEffectorPosition("LFleg", configs[0])
    LH_footPos = fullbody.getEffectorPosition("LHleg", configs[0])
    RF_footPos = fullbody.getEffectorPosition("RFleg", configs[0])
    RH_footPos = fullbody.getEffectorPosition("RHleg", configs[0])

    data['steps'][0]['step'][0]['base_auto']['height'] = configs[0][2]
    data['steps'][1]['step'][1]['footstep']['target']['position'] = RF_footPos[0]
    data['steps'][3]['step'][1]['footstep']['target']['position'] = LH_footPos[0]
    data['steps'][5]['step'][1]['footstep']['target']['position'] = LF_footPos[0]
    data['steps'][7]['step'][1]['footstep']['target']['position'] = RH_footPos[0]


    for i in range(1, len(configs)):
        step = """\
 - step:
    - base_auto:
       average_linear_velocity: 0.5
    - footstep:
       name: 
       target:
        frame: odom
        position:

 - step:
    - base_auto:
       average_linear_velocity: 0.5
        """
        step_data = yaml.load(step)

        if i == (len(configs)-1):
            footPos = fullbody.getEffectorPosition("RFleg", configs[i])
            step_data[0]['step'][1]['footstep']['name'] = 'RF_LEG'
            step_data[0]['step'][1]['footstep']['target']['position'] = footPos[0]

            footPos = fullbody.getEffectorPosition("LHleg", configs[i])
            step_data[0]['step'][1]['footstep']['name'] = 'LH_LEG'
            step_data[0]['step'][1]['footstep']['target']['position'] = footPos[0]

            footPos = fullbody.getEffectorPosition("LFleg", configs[i])
            step_data[0]['step'][1]['footstep']['name'] = 'LF_LEG'
            step_data[0]['step'][1]['footstep']['target']['position'] = footPos[0]

            footPos = fullbody.getEffectorPosition("RHleg", configs[i])
            step_data[0]['step'][1]['footstep']['name'] = 'RH_LEG'
            step_data[0]['step'][1]['footstep']['target']['position'] = footPos[0]

        movedLeg = fullbody.getContactsVariations(i-1, i)
        if movedLeg[0] == 'LFleg':
            footPos = fullbody.getEffectorPosition("LFleg", configs[i])
            step_data[0]['step'][1]['footstep']['name'] = 'LF_LEG'
            step_data[0]['step'][1]['footstep']['target']['position'] = footPos[0]
        elif movedLeg[0] == 'LHleg':
            footPos = fullbody.getEffectorPosition("LHleg", configs[i])
            step_data[0]['step'][1]['footstep']['name'] = 'LH_LEG'
            step_data[0]['step'][1]['footstep']['target']['position'] = footPos[0]
        elif movedLeg[0] == 'RFleg':
            footPos = fullbody.getEffectorPosition("RFleg", configs[i])
            step_data[0]['step'][1]['footstep']['name'] = 'RF_LEG'
            step_data[0]['step'][1]['footstep']['target']['position'] = footPos[0]
        elif movedLeg[0] == 'RHleg':
            footPos = fullbody.getEffectorPosition("RHleg", configs[i])
            step_data[0]['step'][1]['footstep']['name'] = 'RH_LEG'
            step_data[0]['step'][1]['footstep']['target']['position'] = footPos[0]



        data['steps'].extend(step_data)
    stream = file(filename+'.yaml', 'w')
    yaml.dump(data, stream, default_flow_style=False)
    print(('saved .yaml file as '+filename+'.yaml'))

