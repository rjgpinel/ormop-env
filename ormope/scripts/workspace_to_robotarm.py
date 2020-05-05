import click
import os
import pickle


def transform_todict(input_path):
    with open("input_path", "r") as f:
        workspace = pkl.load(f)
        ws_dict = workspace.__dict__

        change_dict(ws_dict, "x_pos", "centers_position_x")
        change_dict(ws_dict, "z_pos", "centers_position_z")
        ws_dict["y_pos"] = [0] * ws_dict["number_of_obstacles"]

        change_dict(ws_dict, "sides_x", "x_side")
        change_dict(ws_dict, "sides_z", "z_side")
        ws_dict["y_side"] = [0.01] * ws_dict["number_of_obstacles"]

        ws_dict["x_axis_rotation"] = [0.0] * ws_dict["number_of_obstacles"]
        ws_dict["z_axis_rotation"] = [0.0] * ws_dict["number_of_obstacles"]

        del ws_dict["rays"]

        return ws_dict

def change_dict(wd_dict, from_key, to_key):
    ws_dict[from_key] = ws_dict[to_key]
    del ws_dict[to_key]


@click.command()
@click.option("-i", "--input_path", "string", default="./")
@click.Option("-o", "--output_path", "string", default="./")
def run(input_path, output_path):
    if not os.path.exists(output_path):
        os.makedirs(output_path)

    for f in os.listdir(input_path):
        if f.endswith(".pkl"):
            old_name = f[:-4]
            new_name = "{}_dict.pkl".format(old_name)
            ws_dict = transform_todict(f)
            with open(output_path, 'wb') as output:
                pkl.dump(ws_dict, output)

