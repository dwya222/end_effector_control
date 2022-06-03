#!/usr/bin/env python3

from demo_interface import DemoInterface

def main():
  mover = DemoInterface()
  mover.listen_for_point()

if __name__ == '__main__':
  main()
