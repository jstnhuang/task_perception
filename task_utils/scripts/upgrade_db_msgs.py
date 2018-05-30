#! /usr/bin/env python
"""Upgrades messages stored in MongoDB
"""

import copy
import pprint
from pymongo import MongoClient


def blank_twist():
    twist = {
        'linear': {
            'x': 0,
            'y': 0,
            'z': 0
        },
        'angular': {
            'x': 0,
            'y': 0,
            'z': 0
        }
    }
    return twist


def add_object_twists_to_demonstrations(db):
    for demonstration in db.demonstrations.find():
        for event in demonstration['events']:
            event['object_twist'] = blank_twist()
        db.demonstrations.replace_one({'name': demonstration['name']}, demonstration)

def add_object_twists_to_object_states(db):
    for demo_states in db.demo_states.find():
        name = demo_states['name']
        for state in demo_states['demo_states']:
            for object_state in state['object_states']:
                object_state['twist'] = blank_twist()
        db.demo_states.replace_one({'name': demo_states['name']}, demo_states)


def main():
    client = MongoClient()
    db = client.pbi
    #add_object_twists_to_demonstrations(db)
    #add_object_twists_to_object_states(db)


if __name__ == '__main__':
    main()
