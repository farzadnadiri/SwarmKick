#!/usr/bin/env python3
"""
swarm_4players_unique_roles_manualfail_noshift.py

A 2D PyGame demo of a 4-player soccer scenario with unique roles
(attacker, midfielder, defender, goalie) assigned each cycle, random start positions,
and *manual* fail/restore:

- Press 1..4 to toggle that player's health (0 => fail, 100 => restore).
- Press R to reset the entire game (random positions).
- Press ESC to quit.

No automatic random failure.

CHANGE: Now, whenever a robot fails/restores, we immediately reassign roles
to ensure there is always an attacker.
"""

import pygame
import sys
import math
import random
import itertools
from collections import defaultdict

# -------------------- GLOBAL CONSTANTS -------------------------------------
SCREEN_WIDTH  = 900
SCREEN_HEIGHT = 600
FPS           = 30

ROLES = ["attacker", "midfielder", "defender", "goalie"]

# Each role's "home zone" if not near the ball
HOME_ZONES = {
    "attacker":   (SCREEN_WIDTH*0.65, SCREEN_HEIGHT*0.5),
    "midfielder": (SCREEN_WIDTH*0.45, SCREEN_HEIGHT*0.5),
    "defender":   (SCREEN_WIDTH*0.25, SCREEN_HEIGHT*0.5),
    "goalie":     (SCREEN_WIDTH*0.05, SCREEN_HEIGHT*0.5)
}

MAX_SPEED = 2.0

# RL parameters (pass vs. dribble)
ALPHA_RL    = 0.1
GAMMA_RL    = 0.9
EPSILON_RL  = 0.05

# Communication dictionary: name -> {pos, dist_ball, battery, etc.}
TEAM_BROADCAST = {}

# Offense/Defense switch timing (ms)
PHASE_SWITCH_INTERVAL = 12000

def dist(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def vec_len(a):
    return math.hypot(a[0], a[1])

def vec_normalize(v):
    l = vec_len(v)
    return (v[0]/l, v[1]/l) if l>1e-9 else (0,0)

# ---------------------------------------------------------------------------
# SIMPLE Q-AGENT: pass/dribble
# ---------------------------------------------------------------------------
class SimpleQAgent:
    def __init__(self):
        self.Q = defaultdict(float)
        self.alpha = ALPHA_RL
        self.gamma = GAMMA_RL
        self.eps   = EPSILON_RL
        self.actions = ["pass", "dribble"]

    def get_action(self, state):
        if random.random() < self.eps:
            return random.choice(self.actions)
        else:
            qvals = [self.Q[(state, a)] for a in self.actions]
            mx = max(qvals)
            idx = qvals.index(mx)
            return self.actions[idx]

    def update(self, state, action, reward, next_state):
        old_q = self.Q[(state, action)]
        next_vals = [self.Q[(next_state, a)] for a in self.actions]
        max_next = max(next_vals) if next_vals else 0.0
        new_q = old_q + self.alpha*(reward + self.gamma*max_next - old_q)
        self.Q[(state, action)] = new_q

# ---------------------------------------------------------------------------
# TEAM-WIDE ROLE ASSIGNMENT
# ---------------------------------------------------------------------------
def assign_roles_priority(players, game_phase, ball):
    """
    Assign roles based on strict priority:
      - 1 healthy => attacker
      - 2 healthy => +goalie
      - 3 healthy => +defender
      - 4 healthy => +midfielder
    Extra healthy players => also 'midfielder' (or any leftover role).

    We pick the "best" player for each needed role, using a simple fitness function.
    """

    # 1) Mark failed
    assignments = {}
    for p in players:
        if p.health <= 0:
            assignments[p.name] = "failed"

    # 2) Gather healthy players
    healthy_players = [p for p in players if p.health > 0]
    n = len(healthy_players)
    if n == 0:
        # No healthy players => done
        return assignments

    # 3) Determine which roles we "need" in priority order
    # E.g., if n=2 => roles_needed = ["attacker","goalie"]
    roles_needed = []
    if n >= 1:
        roles_needed.append("attacker")
    if n >= 2:
        roles_needed.append("goalie")
    if n >= 3:
        roles_needed.append("defender")
    if n >= 4:
        roles_needed.append("midfielder")

    # 4) Define a small helper for "role fitness"
    def role_fitness(player, role):
        d = math.hypot(player.x - ball.x, player.y - ball.y)
        batt = player.battery
        
        if role == "attacker":
            # attacker => close to ball + good battery
            return (1.0 / (d + 0.1)) * (batt / 100.0)
        elif role == "goalie":
            # better if defense phase
            return ((0.8 if game_phase == "defense" else 0.3) * (batt / 100.0))
        elif role == "defender":
            # better if defense
            return ((1.0 if game_phase == "defense" else 0.5) * (batt / 100.0))
        elif role == "midfielder":
            # moderate usage
            return (0.8 * (batt / 100.0))
        else:
            return 0.0

    # 5) Assign exactly one player for each needed role, in priority order
    for role in roles_needed:
        if not healthy_players:
            break  # no more players to assign

        best_pl = None
        best_val = -9999
        for hp in healthy_players:
            val = role_fitness(hp, role)
            if val > best_val:
                best_val = val
                best_pl = hp
        # assign that role
        if best_pl is not None:
            assignments[best_pl.name] = role
            healthy_players.remove(best_pl)

    # 6) Any leftover healthy players => "midfielder"
    for hp in healthy_players:
        assignments[hp.name] = "midfielder"

    return assignments

# ---------------------------------------------------------------------------
# BALL
# ---------------------------------------------------------------------------
class Ball:
    def __init__(self):
        self.x = 450
        self.y = 300
        self.radius = 10
        self.color = (255,165,0)

    def reset(self):
        self.x = 450
        self.y = 300

    def draw(self, screen):
        pygame.draw.circle(screen, self.color, (int(self.x), int(self.y)), self.radius)

# ---------------------------------------------------------------------------
# PLAYER
# ---------------------------------------------------------------------------
class Player:
    def __init__(self, name, color):
        self.name   = name
        self.color  = color
        self.radius = 15
        self.reset_data()

        self.rl = SimpleQAgent()
        self.last_state  = None
        self.last_action = None

    def reset_data(self):
        """Random position, healthy again."""
        self.x = random.randint(50, SCREEN_WIDTH//2 )
        self.y = random.randint(50, SCREEN_HEIGHT - 50)
        self.vx = 0.0
        self.vy = 0.0
        self.health = 100
        self.battery = 100.0
        self.current_role = "midfielder"

    def broadcast(self):
        TEAM_BROADCAST[self.name] = {
            'pos': (self.x, self.y),
            'health': self.health,
            'battery': self.battery
        }

    def toggle_fail(self):
        """If healthy => fail. If failed => restore."""
        if self.health > 0:
            self.health = 0
            print(f"{self.name} manually FAILED!")
        else:
            self.health = 100
            print(f"{self.name} manually RESTORED!")

    def step(self, assigned_role, ball):
        if self.health <= 0:
            self.current_role = "failed"
            return
        self.current_role = assigned_role

        # Distance to ball
        d_ball = dist((self.x, self.y), (ball.x, ball.y))
        has_ball = "yes" if d_ball < (self.radius + ball.radius + 5) else "no"

        # ------------------------------------------------
        # If attacker AND has the ball => override target
        # ------------------------------------------------
        if assigned_role == "attacker":
            if has_ball == "yes":
                # move attacker to the vertical center of right side
                base_target = (SCREEN_WIDTH - self.radius - 5, SCREEN_HEIGHT // 2)
            else:
                # chase ball if we don't have it
                base_target = (ball.x, ball.y)
        elif assigned_role in HOME_ZONES:
            base_target = HOME_ZONES[assigned_role]
        else:
            base_target = (self.x, self.y)

        # RL micro decision: pass/dribble
        dist_bin = "close" if d_ball < 80 else "far"
        state = (assigned_role, dist_bin, has_ball)
        action = self.rl.get_action(state)

        # RL update from last state/action
        if self.last_state and self.last_action:
            reward = 0.0
            # Example small reward for attacker having the ball
            if self.last_state[0] == "attacker" and self.last_state[2] == "yes":
                reward += 0.2
            self.rl.update(self.last_state, self.last_action, reward, state)

        self.last_state  = state
        self.last_action = action

        # Compute direction to base_target
        dx = base_target[0] - self.x
        dy = base_target[1] - self.y
        l  = math.hypot(dx, dy)
        if l > 1e-9:
            nx = dx / l
            ny = dy / l
        else:
            nx = ny = 0

        spx = nx * MAX_SPEED
        spy = ny * MAX_SPEED
        if action == "pass":
            spx *= 0.6
            spy *= 0.6

        # Move
        self.vx, self.vy = spx, spy
        self.x += self.vx
        self.y += self.vy

        # Boundary clamp
        if self.x <  self.radius:               self.x = self.radius
        if self.x > SCREEN_WIDTH - self.radius: self.x = SCREEN_WIDTH - self.radius
        if self.y <  self.radius:               self.y = self.radius
        if self.y > SCREEN_HEIGHT - self.radius:self.y = SCREEN_HEIGHT - self.radius

    def draw(self, screen):
        color = (80,80,80) if self.health <= 0 else self.color
        pygame.draw.circle(screen, color, (int(self.x), int(self.y)), self.radius)
        font = pygame.font.SysFont(None, 16)
        txt  = font.render(f"{self.name}-{self.current_role}", True, (255,255,255))
        screen.blit(txt, (self.x - 20, self.y - 30))

# ---------------------------------------------------------------------------
# DRAW FIELD
# ---------------------------------------------------------------------------
def draw_field(screen):
    field_color = (50,120,50)
    pygame.draw.rect(screen, field_color, (0,0,SCREEN_WIDTH,SCREEN_HEIGHT))
    # center line
    midx = SCREEN_WIDTH // 2
    pygame.draw.line(screen, (255,255,255), (midx,0), (midx,SCREEN_HEIGHT), 2)
    # circle
    pygame.draw.circle(screen, (255,255,255), (midx, SCREEN_HEIGHT//2), 60, 2)
    # penalty boxes
    pygame.draw.rect(screen, (255,255,255), (0, SCREEN_HEIGHT//2 - 100, 100, 200), 2)
    pygame.draw.rect(screen, (255,255,255), (SCREEN_WIDTH - 100, SCREEN_HEIGHT//2 - 100, 100, 200), 2)

# ---------------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------------
def main():
    pygame.init()
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption("4 Players - Unique Roles w/Manual Fail, No SHIFT")
    clock = pygame.time.Clock()

    # Create 4 players
    players = []
    for i in range(4):
        c = (0, 140 + 30*i, 255)
        name = f"P{i+1}"
        p = Player(name, c)
        players.append(p)

    # Ball
    ball = Ball()

    GOAL_X = SCREEN_WIDTH - 30

    phase = "offense"
    switch_timer = 0
    
    # Reassign roles only every 3 seconds
    role_timer = 99999  # Force immediate assignment on first loop
    ROLE_REASSIGN_INTERVAL = 5000  # 3 seconds in ms

    # We'll store the current assignments here and update them only when needed
    assignments = {}

    def reset_game():
        """Reset everything with random positions for players and center ball."""
        for p in players:
            p.reset_data()
        ball.reset()
        print("Game has been reset. (Players have new random positions)")

    running = True
    while running:
        dt = clock.tick(FPS)
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.KEYDOWN:
                # ESC => quit
                if event.key == pygame.K_ESCAPE:
                    running = False
                # R => reset
                elif event.key == pygame.K_r:
                    reset_game()
                # 1..4 => toggle fail for that player
                elif event.key == pygame.K_1:
                    players[0].toggle_fail()
                    assignments = assign_roles_priority(players, phase, ball)  # <-- NEW
                    role_timer = 0                                    # <-- NEW
                elif event.key == pygame.K_2:
                    players[1].toggle_fail()
                    assignments = assign_roles_priority(players, phase, ball)  # <-- NEW
                    role_timer = 0                                    # <-- NEW
                elif event.key == pygame.K_3:
                    players[2].toggle_fail()
                    assignments = assign_roles_priority(players, phase, ball)  # <-- NEW
                    role_timer = 0                                    # <-- NEW
                elif event.key == pygame.K_4:
                    players[3].toggle_fail()
                    assignments = assign_roles_priority(players, phase, ball)  # <-- NEW
                    role_timer = 0                                    # <-- NEW

        # Update timers
        switch_timer += dt
        role_timer   += dt

        # Switch phase (offense/defense) every 12s
        if switch_timer > PHASE_SWITCH_INTERVAL:
            switch_timer = 0
            phase = "defense" if phase == "offense" else "offense"
            # Force immediate role re-assignment on phase change
            assignments = assign_roles_priority(players, phase, ball)
            role_timer  = 0  # reset

        # Reassign roles every 3 seconds if not triggered by a fail event above
        if role_timer > ROLE_REASSIGN_INTERVAL:
            assignments = assign_roles_priority(players, phase, ball)
            role_timer = 0

        # 1) broadcast
        TEAM_BROADCAST.clear()
        for p in players:
            p.broadcast()

        # 2) step players using the *current* assignments
        for p in players:
            # If no assignments yet, default to "midfielder"
            assigned_role = assignments.get(p.name, "midfielder")
            p.step(assigned_role, ball)

        # 3) check goal
        for p in players:
            if p.health > 0:
                d = dist((p.x, p.y), (ball.x, ball.y))
                if d < (p.radius + ball.radius):
                    # move ball
                    ball.x = p.x + (p.x - ball.x)*0.1
                    ball.y = p.y + (p.y - ball.y)*0.1
                    # check if ball crosses the goal line
                    if ball.x > GOAL_X:
                        print(f"GOAL by {p.name} => {p.current_role}")
                        ball.reset()
                        break

        # 4) draw
        draw_field(screen)
        for p in players:
            p.draw(screen)
        ball.draw(screen)

        font = pygame.font.SysFont(None, 24)
        txt  = font.render(
            f"PHASE: {phase.upper()} | [1..4] => toggle fail | [R] = reset",
            True, (255,255,255)
        )
        screen.blit(txt, (10,10))

        pygame.display.flip()

    pygame.quit()
    sys.exit()

if __name__ == "__main__":
    main()
