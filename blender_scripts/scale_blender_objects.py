# fmt: off
def print_properties_of_selected():
    for obj in C.selected_objects:
        print_obj_properties(obj)

def print_obj_properties(obj):
    if obj.type == "MESH":
        if obj.ambf_object_type == "RIGID_BODY":
            print(f"Object Name; {obj.name}")
            print(f"rigid_body_linear_inertial_offset; {obj.ambf_rigid_body_linear_inertial_offset}")
            print(f"Number of collision primitives; {len(obj.ambf_collision_shape_prop_collection)}")
            for prop_tuple in obj.ambf_collision_shape_prop_collection.items():
                shape_prop = prop_tuple[1]
                print(f"linear offset; {shape_prop.ambf_collision_shape_linear_offset}")
                if shape_prop.ambf_collision_shape == "BOX":
                    print(f"BOX dims; {shape_prop.ambf_collision_shape_xyz_dims}")
                elif shape_prop.ambf_collision_shape in ["CONE", "CYLINDER", "CAPSULE", "SPHERE"]:
                    print(f"radius; {shape_prop.ambf_collision_shape_radius}")
                    print(f"height; {shape_prop.ambf_collision_shape_height}")

def ambf_rigid_body_set_scale(object_handle, scale):
    if object_handle.type == "MESH":
        if object_handle.ambf_object_type == "RIGID_BODY":
            object_handle.ambf_rigid_body_linear_inertial_offset = (
                object_handle.ambf_rigid_body_linear_inertial_offset * scale
            )
            print(f"Modifying: {object_handle.name}")
            for (
                prop_tuple
            ) in object_handle.ambf_collision_shape_prop_collection.items():
                shape_prop_group = prop_tuple[1]
                collision_shape_set_scale(shape_prop_group, scale)


def collision_shape_set_scale(shape_prop, scale):

    ## Not sure why adnan added this check. All collision shapes have null pointers
    # coll_shape_obj_handle = shape_prop.ambf_collision_shape_pointer
    # if coll_shape_obj_handle is None:
    #     return

    if shape_prop.ambf_collision_shape == "BOX":
        print("Setting BOX scale", scale)
        shape_prop.ambf_collision_shape_xyz_dims = (
            shape_prop.ambf_collision_shape_xyz_dims * scale
        )
    elif shape_prop.ambf_collision_shape in ["CONE", "CYLINDER", "CAPSULE", "SPHERE"]:
        shape_prop.ambf_collision_shape_radius = (
            shape_prop.ambf_collision_shape_radius * scale
        )
        shape_prop.ambf_collision_shape_height = (
            shape_prop.ambf_collision_shape_height * scale
        )
    shape_prop.ambf_collision_shape_linear_offset = (
        shape_prop.ambf_collision_shape_linear_offset * scale
    )
    return shape_prop

## SCRIPTS
def set_scale_of_selected(scale):
    for o in C.selected_objects:
        ambf_rigid_body_set_scale(o, scale)


