import argparse
import os
import platform
import socket
import subprocess

import docker
from dt_shell import DTCommandAbs, DTShell, dtslogger
from dt_shell.env_checks import check_docker_environment

#this is in ~/.dt-shell/commands-multi/daffy
import sys
sys.path.append(os.path.expanduser("~/.dt-shell/commands-multi/daffy"))
#print(sys.path)
from utils.cli_utils import start_command_in_subprocess
from utils.docker_utils import remove_if_running, pull_if_not_exist, build_if_not_exist
from utils.networking_utils import get_duckiebot_ip


CONTROLLER_COMMAND = "roslaunch controller_interface controller_interface.launch veh:={veh}"
process= subprocess.Popen(["git", "rev-parse", "--abbrev-ref", "HEAD"], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
BRANCH = process.communicate()[0].decode("utf-8").strip()
DEFAULT_IMAGE = "dt-low-level-control:" + BRANCH
AVAHI_SOCKET = "/var/run/avahi-daemon/socket"


class DTCommand(DTCommandAbs):
    @staticmethod
    def command(shell: DTShell, args):
        prog = "dts duckiebot start_navigation DUCKIEBOT_NAME"
        usage = """
Start navigation:
    %(prog)s
"""

        parser = argparse.ArgumentParser(prog=prog, usage=usage)

        parser.add_argument("--network", default="host", help="Name of the network to connect to")
        parser.add_argument("--sim", action="store_true", default=False, help="are we running in simulator?")
        parser.add_argument(
            "--image",
            default=DEFAULT_IMAGE,
            help="The base image, probably don't change the default",
        )

        parser.add_argument("hostname", default=None, help="Name of the Duckiebot to control")

        parsed_args = parser.parse_args(args)

        if parsed_args.sim:
            duckiebot_ip = "sim"
        else:
            duckiebot_ip = get_duckiebot_ip(duckiebot_name=parsed_args.hostname)

        network_mode = parsed_args.network

        run_controller(
            parsed_args.hostname, parsed_args.image, duckiebot_ip, network_mode, parsed_args.sim
        )

# if it's the CLI may as well run it on the robot itself.
def run_controller(hostname, image, duckiebot_ip, network_mode, sim):
    if sim:
        duckiebot_client = check_docker_environment()
    else:
        duckiebot_client = docker.DockerClient("tcp://" + duckiebot_ip + ":2375")
    print(duckiebot_client.images.list())
    container_name = "auto_drive_%s" % hostname
    remove_if_running(duckiebot_client, container_name)
    env = set_default_env(hostname, duckiebot_ip)

    dtslogger.info("Running %s on localhost with environment vars: %s" % (container_name, env))

    #if not sim:
    #    image = image.replace("amd64", "arm32v7")

    params = {
        "image": image,
        "name": container_name,
        "network_mode": network_mode,
        "environment": env,
        "privileged": True,
        "stdin_open": True,
        "tty": True,
        "command": CONTROLLER_COMMAND.format(veh=hostname),
        "detach": True,
    }

    build_if_not_exist(
        client=duckiebot_client,
        image_path=os.path.expanduser("~/Duckietown/dt-low-level-control"),
        tag=BRANCH
    )
    print("Do you see your build?")
    print(duckiebot_client.images.list())
    duckiebot_client.containers.run(**params)

    cmd = "docker %s attach %s" % ("-H %s.local" % hostname if not sim else "", container_name)
    dtslogger.info("attach command: %s" % cmd)
    start_command_in_subprocess(cmd)


def set_default_env(hostname, ip):
    env = {
        "HOSTNAME": hostname,
        "ROS_MASTER": hostname,
        "VEHICLE_NAME": hostname,
        "VEHICLE_IP": ip,
        "ROS_MASTER_URI": "http://%s:11311" % ip,
    }
    return env

if __name__ == "__main__":
    #client=docker.DockerClient("tcp://" + duckiebot_ip + ":2375")
    #client.images.get(image_name)
    DTCommand.command("", ["charlie"])