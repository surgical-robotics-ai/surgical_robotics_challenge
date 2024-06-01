# Kinematic Config Files

## Included dVRK Patient Side Manipulator

| # | Folder Name       | Description                                                 |
|---|-------------------|-------------------------------------------------------------|
| 1 | `psm_400006.json` | Classic dVRK PSM with classic Larger Needle Driver attached |
| 1 | `psm_420006.json` | Classic dVRK PSM with Si Larger Needle Driver attached      |

## Included dVRK Instruments

| # | Folder Name                       | Description                                         |
|---|-----------------------------------|-----------------------------------------------------|
| 1 | `Large_Needle_Driver_400006.json` | Large Needle Driver for da Vinci surgical system    |
| 1 | `Large_Needle_Driver_420006.json` | Large Needle Driver for da Vinci Si surgical system |

*** Note: both psm and instrument json files just following the format of the dVRK repository. Nevertheless, 
some values are different due to the customized kinematic design for surgical robotics challenge.

## Folder Architecture

```bash
.<surgical_robotics_challenge scripts folder>
├── kinematics              # Kinematic information for whole surgical robotics challenge
│   ├── config              # the parent folder of all configuration files, mimic the dVRK Repo structure 
│   │   ├── kinematic       # PSM config files 
│   │   ├── tool            # surgical instrument config files
│   ├── ...                 # FK/IK scripts
├── ...                     # Other files
```

## Config JSON file Architecture

```bash
...
"DH": {
    "convention": "modified",   // DH paremeter type (modified or standard)           
    "joints": [
        {
            "name": ... ,  // name of the joint
            "alpha":  1.5708, "A":  0.0000, 
            "theta":  0.0000, "D":  0.0000, // DH parameters
            "type": "revolute",  // joint type
            ...
            "offset":  1.5708, // DH offset
            "qmin": -1.588,  // lower limit
            "qmax":  1.588,  // upper limit               
            ...
        },
...
```