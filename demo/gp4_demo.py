import gymnasium
import manipulator_mujoco
import threading
import glfw

# Create the environment with rendering in human mode
env = gymnasium.make('manipulator_mujoco/GP4Env-v0', render_mode='human')

# Reset the environment with a specific seed for reproducibility
observation, info = env.reset(seed=42)
mode = 0
def take_input():
    global mode
    while(True):
        n = input()
        try:
            mode = float(n)
        except:
            print("wrong input")
            continue
t1 = threading.Thread(target=take_input)
t1.daemon = True
t1.start()
while True:
    # Choose a random action from the available action space
    action = 0
    # Take a step in the environment using the chosen action
    observation, reward, terminated, truncated, info = env.step(mode)

    # Check if the episode is over (terminated) or max steps reached (truncated)
    if terminated or truncated:
        # If the episode ends or is truncated, reset the environment
        observation, info = env.reset()

# Close the environment when the simulation is done
env.close()
