import json
import socket
import time

UDP_IP = "127.0.0.1"
CONTROL_PORT = 5005
STATE_PORT = 5006

send_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
recv_sock.bind(("0.0.0.0", STATE_PORT))

print("Multi-aircraft controller started")
last_print_time = 0.0


def recv_aircraft_list():
    data, _ = recv_sock.recvfrom(65535)
    payload = json.loads(data.decode("utf-8"))
    if isinstance(payload, dict) and isinstance(payload.get("aircraft"), list):
        return payload["aircraft"]
    if isinstance(payload, dict):
        return [payload]
    return []


def build_uav_command(name, aircraft):
    speed_mps = aircraft.get("speed_mps", 0.0)
    pitch_state = aircraft.get("pitch", 0.0)

    target_pitch = 5.0
    throttle = 1.0 if speed_mps < 150.0 else 0.6

    pitch_error = target_pitch - pitch_state
    pitch_cmd = max(-1.0, min(1.0, pitch_error * 0.1))

    return {
        "aircraft_name": name,
        "roll": 0.0,
        "pitch": -pitch_cmd,
        "yaw": 0.0,
        "throttle": throttle,
    }


while True:
    aircraft_list = recv_aircraft_list()
    if not aircraft_list:
        continue

    should_print = (time.monotonic() - last_print_time) >= 1.0
    if should_print:
        last_print_time = time.monotonic()
        print(f"received {len(aircraft_list)} aircraft")

    uav_aircraft = []
    for aircraft in aircraft_list:
        name = aircraft.get("aircraft_name", "unknown")
        team = aircraft.get("team")
        speed_mps = aircraft.get("speed_mps")
        throttle = aircraft.get("throttle")
        x = aircraft.get("x")
        y = aircraft.get("y")
        z = aircraft.get("z")
        pitch = aircraft.get("pitch")
        roll = aircraft.get("roll")
        yaw = aircraft.get("yaw")
        weapons = aircraft.get("weapons", {})
        bullet_ammo = weapons.get("bullet_ammo")
        rocket_ammo = weapons.get("rocket_ammo")

        if should_print:
            print(
                f"[{name}] "
                f"pos=({x}, {y}, {z}) "
                f"speed={speed_mps} "
                f"pitch={pitch} roll={roll} yaw={yaw} "
                f"throttle={throttle} "
                f"team={team} "
                f"bullet_ammo={bullet_ammo} rocket_ammo={rocket_ammo}"
            )

        if "UAV" in name:
            uav_aircraft.append(aircraft)

    commands = []
    for aircraft in uav_aircraft[:2]:
        commands.append(build_uav_command(aircraft["aircraft_name"], aircraft))

    if not commands:
        if should_print:
            print("No UAV aircraft found to control")
        continue

    payload = {"commands": commands}
    send_sock.sendto(json.dumps(payload).encode("utf-8"), (UDP_IP, CONTROL_PORT))
    if should_print:
        print("sent:", payload)
