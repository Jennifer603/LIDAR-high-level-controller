import asyncio  # write concurrent code using the async/await syntax.
import agility
import agility.messages as msgs
import time

def error_handler(msg):
    # Message handler that prints out error/warning messages
    print(msg.type, msg.info)
    return True


# Main function, starts all other tasks
async def main(address, port, connect_timeout):
    # Connect to simulator on localhost:8080 by default
    async with agility.JsonApi(address = address, port = port, connect_timeout = connect_timeout) as api:
        # Install default error/warning handlers that just print messages
        api.handle('error', error_handler)
        api.handle('warn', error_handler)
        info = await api.query(msgs.GetRobotInfo())
        print(f"Robot name: {info.robot_name}")
        await asyncio.sleep(1)  
        # Request the change action command privilege
        await api.request_privilege('change-action-command')
        # add objects for the environment setups.
        await api.send(msgs.AddObject(attributes={
            "name":"my-box",
            "box-geometry":[0.4, 0.4, 0.4],
            "mass":1.0,
            "pickable":True
        }))
        # or:
        # await api.send([
        #     "add-object",
        #     {
        #         "attributes": {
        #         "name": "my-box",
        #         "box-geometry": [0.4, 0.4, 0.4],
        #         "mass": 1,
        #         "pickable": True
        #         }
        #     }
        # ])
        await api.send([   # add april-tag to the box for the perception
            "add-object",
            {
                "attributes": { "april-tag-id": 0 },
                "transform": { "xy": [0.2, 0] },
                "relative-to": { "owned-object-name": "my-box" }
            }
        ])
        await api.send([
        "add-object",
            {
                "attributes": {
                "name": "pick-table",
                "box-geometry": [0.4, 1, 0.05],
                "polygon": [[0.2, 0.5], [0.2, -0.5], [-0.2, -0.5], [-0.2, 0.5]],
                "keep-out": True
                },
                "transform": { "xyz": [5.5, 0, 0.7] }
            }
        ])
        await api.send([
        "add-object",
            {
                "attributes": {
                "name": "place-table",
                "box-geometry": [0.4, 1, 0.05],
                "polygon": [[0.2, 0.5], [0.2, -0.5], [-0.2, -0.5], [-0.2, 0.5]],
                "keep-out": True
                },
                "transform": { "xyz": [-1.5, 0, 0.7] }
            }
        ])
        # Start printing out timestamped position data for the robot
        # run the concurrent monitoring program
        # monitor_task = asyncio.ensure_future(print_position(api)) # Python 3.6-
        # ensure_future/create_task is developed by asyncio for the multi-task applications
        monitor_task = asyncio.create_task(print_position(api)) # Python 3.7+
        # Sends an action-stand message with an explicit base pose
        # we can directly control the end-effector.
        # waypoints -> array of pose
        # await api.wait_action(msgs.ActionGoto(target={"axyd": [0, 0, 0.4]}, position_tolerance=0.05, orientation_tolerance=0.1,
        # mobility_parameters={"avoid-obstacles": False, "step-clearance": 0.01,"velocity-max": [0.1, 0.1, 0.1]}),remove_after=False)
        # await asyncio.sleep(5)
        await api.wait_action(msgs.ActionEndEffectorMove(end_effector="left-hand", waypoints=[{"xy":[0.1,0.1]},{"xy":[0.2,0.2]}], reference_frame={"command-frame":"base"}, cyclic=False, max_speed=0.5, duration=1))
        await api.wait_action(msgs.ActionGoto(target={"xy": [6.5, 0]}, position_tolerance=0.05,mobility_parameters={"step-clearance":0.01,"velocity-max":[2,2,1.0]}),remove_after=False)
        await asyncio.sleep(0.5)  
        await api.wait_action(msgs.ActionPick(object={"owned-object-name": "my-box"}))
        # Command the robot to walk forward three meters relative to its
        # current position  -> relative to the current robot frame
        # prefer to using all the coordinates in the world frame
        # obj = await api.get_response(msgs.AddObject( 
        #     # transform={"xyz":[-5,-5,0]},
        #     transform={},
        #     relative_to={'robot-frame': msgs.RobotFrame.BASE},
        #     stored_relative_to={'special-frame': 'world'},
        # ))
        # wait_action is super important
        await api.wait_action(msgs.ActionPlace(pose={"xyz":[0,0,0.24]}, reference_frame={"owned-object-name": "place-table"}, position_tolerance=0.1))
        await asyncio.sleep(0.5)
        await api.wait_action(msgs.ActionPick(object={"owned-object-name": "my-box"}))
        await api.wait_action(msgs.ActionPlace(pose={"xyz":[0,0,0.24]}, reference_frame={"owned-object-name": "pick-table"}, position_tolerance=0.1))
        # await api.wait_action(msgs.ActionGoto(target={'xy': [9.5, 0.5]}))
        # await asyncio.sleep(0.5)
        # await api.wait_action(msgs.ActionGoto(target={"xy": [6.5, 0]}))
# automatically finished

# Task for monitoring robot position
async def print_position(api):
    # Get timestamped robot frame position data
    # put these two actions in a query group
    queries = [
        msgs.GetTimestamp(),
        msgs.GetObjectKinematics(object={'robot-frame': msgs.RobotFrame.BASE}),
    ]

    # Send a periodic query group, clean up query when task is canceled
    # periodic query -> keep printing out messages (second parameter -> print time interval)
    # this can only be used with world-state queries
    async with api.periodic_query(queries, 0.1) as query:
        # When each response is received, print a message to the console
        async for result in query:
            print(f'time: {result[0].run_time}, '
                  f'x: {result[1].transform.rpyxyz[3]}, '
                  f'y: {result[1].transform.rpyxyz[4]}')  # [3] -> x [4] -> y

# Launch simulator
# Change this to the path to the simulator binary on your system
sim_path = '../ar-control'
TOML_file_path = '../example.toml'  # pre-defined obstacle map for the high-level task
address = "127.0.0.1"  # simulator
# address = "10.10.1.1" physical robot
port = 8080
connect_timeout = 100
with agility.Simulator(sim_path, TOML_file_path) as sim:
    # Start running main function
    # asyncio.run(main()) # Python 3.7+
    asyncio.run(main(address, port, connect_timeout))
