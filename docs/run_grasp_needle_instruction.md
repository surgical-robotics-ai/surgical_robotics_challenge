# How to run the scripts for needle grasping

## Active needle grasping (move PSM)

The script is at `scripts/surgical_robotics_challenge/utils/grasp_needle_active.py` 

I hardcoded some statements in the script instead of using arguments:

1. tool_id: Please modify the tool_id at line #70/71
2. grasp_psm: which psm to grasp the needle, modify at line #84/85

After modifying the parameters, you can run the following command in the terminal:

```bash
python3 grasp_needle_active.py
```

## Passive needle grasping (move needle)