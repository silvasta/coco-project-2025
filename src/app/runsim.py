#!/usr/bin/env python3
'''
This is a terminal application able to run a simulation in several different controllers:
No control, DeePC, Hankel (random controller), MPC, Regions and MFD.

It requires:
    - controller mode:
        - parameters for the controller .json format
    - simulation name
    - 
'''

''' IMPORTS '''

import os, sys,curses
current = os.path.dirname(os.path.realpath(__file__))
parent = os.path.dirname(current)
sys.path.append(parent)
from curses import wrapper
from logs.error_handling.exceptions import *
from src.network.network import Network
from tools.utils import pickler
from menu import Menu,SelMenu
from consim import consim
from hansim import hansim
from regsim import regsim
from mfdsim import mfdsim
from thetasim import thetasim
from gensim import gensim
    
def main(stdscr):
    
    curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
    GREEN_ON_BLACK = curses.color_pair(1)
    os.system("source /home/semsepiol/Code/envs/FixT/bin")
    ron = pickler()
    
    ### Welcome screen & Network Creation, Page 1 ###
    stdscr.clear()
    stdscr.addstr(0,20,"Welcome to the CONTROL SUMO environment !",curses.A_REVERSE)
    stdscr.addstr("\n \n")
    stdscr.refresh()
    mainwin = curses.newwin(0,0,5,5)
    mainwin.addstr("This is an application to run easily SUMO simulations. \n")
    mainwin.addstr("If you would like to run a custom simulation, please create a folder in /dep/sumo_files as detailed in the documentation page \n \n")
    mainwin.refresh()

    # Network Selection Menu
    pos = (5,0)
    mainwin.addstr("Select the Network you would like to simulate : ")
    mainwin.refresh()
    avsim = os.listdir("./dep/sumo_files")
    menu = SelMenu(avsim, mainwin, pos)
    name = menu.display()

    try:
        net = Network(name)
        mainwin.erase()
        mainwin.addstr("Network created successfully. \n")
        mainwin.refresh()
    except:
        raise NetworkCreationError(name)
    
    mainwin.clear()
    mainwin.addstr("What would you like to do ? \n")
    mainwin.refresh()
    menu_items = [
        ("Generate sumocfg/trips",gensim),
        ("Retrieve Regions",regsim),
        ("Retrieve MFD",mfdsim),
        ("Retrieve Hankel Matrix",hansim),
        ("Retrieve routing tensor Theta", thetasim),
        ("Control the Simulation", consim),
    ]
    pos = (2,0)
    menu = Menu(menu_items, mainwin,pos,net,name)
    menu.display()

    ### End Screen ###
    stdscr.refresh()
    stdscr.clear()
    stdscr.addstr("Simulation completed. \n")
    stdscr.addstr("The results can be found in the /out folder. \n \n")
    stdscr.addstr("Press any key to exit.")
    stdscr.getkey()
   

if __name__ == "__main__":
    wrapper(main)
    



