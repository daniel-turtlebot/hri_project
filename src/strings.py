import numpy as np

inst_string = "Press Enter to proceed \n Press Esc to quit \n Backspace for prompt"
def get_colour_order():
    colours = ["Red","Pink","Yellow"]
    np.random.shuffle(colours)
    str = " ".join(colours)
    str1 = "Fetch Colours in this Order: \n"
    for l in range(10):
        str += "\n"
    str += "Clocks Ticking.........."
    return str1 + str