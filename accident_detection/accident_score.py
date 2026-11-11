COLLISION_WEIGHT = 0.1
SPEED_WEIGHT = 0.4
DIRECTION_WEIGHT = 0.5
SMOKE_WEIGHT = 1.0


def calculate_accident_score(
        collision,
        speed,
        direction,
        smoke):

    score = 0

    if collision:
        score += COLLISION_WEIGHT

    if speed:
        score += SPEED_WEIGHT

    if direction:
        score += DIRECTION_WEIGHT

    if smoke:
        score += SMOKE_WEIGHT

    return score