import time
from . import logger

def pick_cup(cup_id: str, arm: str):
    """Simulate picking up a cup."""
    logger.log(f"{arm} is picking up {cup_id}")
    time.sleep(1)  # simulate time delay for picking up cup

def pull_espresso(cup_id: str, arm: str):
    """Simulate pulling an espresso shot."""
    logger.log(f"{arm} is pulling an espresso for {cup_id}")
    time.sleep(2)  # simulate time delay for pulling espresso shot

def steam_milk(cup_id: str, arm: str):
    """Simulate steaming milk."""
    logger.log(f"{arm} is steaming milk for {cup_id}")
    time.sleep(3)  # simulate time delay for steaming milk

def pour_milk(cup_id: str, arm: str):
    """Simulate pouring milk into the cup."""
    logger.log(f"{arm} is pouring milk into {cup_id}")
    time.sleep(1)  # simulate time delay for pouring milk

def add_chocolate_syrup(cup_id: str, arm: str):
    """Simulate adding chocolate syrup to the cup."""
    logger.log(f"{arm} is adding chocolate syrup to {cup_id}")
    time.sleep(1)  # simulate time delay for adding syrup

def heat_water(cup_id: str, arm: str):
    """Simulate heating water."""
    logger.log(f"{arm} is heating water for {cup_id}")
    time.sleep(2)  # simulate time delay for heating water

def pour_water(cup_id: str, arm: str):
    """Simulate pouring hot water into the cup."""
    logger.log(f"{arm} is pouring hot water into {cup_id}")
    time.sleep(1)  # simulate time delay for pouring water

def sprinkle_cocoa(cup_id: str, arm: str):
    """Simulate sprinkling cocoa on top of the drink."""
    logger.log(f"{arm} is sprinkling cocoa on {cup_id}")
    time.sleep(1)  # simulate time delay for sprinkling cocoa

def serve(cup_id: str, arm: str):
    """Simulate serving the completed drink."""
    logger.log(f"{arm} is serving {cup_id}")
    time.sleep(1)  # simulate time delay for serving the drink
