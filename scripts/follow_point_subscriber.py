#!/usr/bin/env python

from demo_interface import DemoInterface

def main():
  mover = DemoInterface()
  mover.listen_for_point()

if __name__ == '__main__':
  main()
