from simulation.environment import SoccerEnvironment

if __name__ == "__main__":
    env = SoccerEnvironment()
    try:
        for _ in range(10000):
            env.step_simulation()
    finally:
        env.close()
