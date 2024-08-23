import bpy
import bpy.props

def set_modifier_property_by_ui_name(obj, mod_name, ui_property_name, value):
    mod = obj.modifiers.get(mod_name)
    if mod:
        # Iterate over the modifier's RNA properties
        for prop in mod.node_group.nodes:
            print(prop.name)
            if prop.name == ui_property_name:
                setattr(mod, prop.identifier, value)
                print(f"Set {ui_property_name} to {value} on {mod_name}")
                break
        else:
            print(f"Property '{ui_property_name}' not found on modifier '{mod_name}'")
    else:
        print(f"Modifier '{mod_name}' not found on object '{obj.name}'")

if __name__ == "__main__":

    # Get selected object
    obj = bpy.context.active_object

    modifier = obj.modifiers.get("Skin")

    for key in modifier.keys():
        # Skip Blender's internal properties like "_RNA_UI"
        if key.startswith("_"):
            continue

        # Print the dynamically assigned name and its value
        print(f"Property: {key}, Value: {modifier[key]}")