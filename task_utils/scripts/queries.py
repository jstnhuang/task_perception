#! /usr/bin/env python

import pprint
from pymongo import MongoClient

def get_number_of_set_pose_events(db):
    for demonstration in db.demonstrations.find():
        num_pose_events = 0
        num_spawn_events = 0
        for event in demonstration['events']:
            if event['type'] == 'set object pose':
                num_pose_events += 1
            if event['type'] == 'spawn object':
                num_spawn_events += 1
        num_pose_events -= num_spawn_events
        num_frames = demonstration['frame_count']
        print '{} {} {} {}%'.format(demonstration['name'], num_pose_events, num_frames, 100.0 * num_pose_events/num_frames )

def main():
    client = MongoClient()
    db = client.pbi
    get_number_of_set_pose_events(db)

if __name__ == '__main__':
    main()
