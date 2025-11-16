
def detect_cup_grippper():

    try:
        return True
    except Exception as e:
        return False

SEQUENCES = {
    'detect_cup_grippper':detect_cup_grippper,
}