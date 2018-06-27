#! /usr/bin/env python

import pprint
from pymongo import MongoClient

def get_number_of_set_pose_events(db):
    print 'Pose events'
    demo_breakdown = {}
    for demonstration in db.demonstrations.find():
        num_pose_events = 0
        num_spawn_events = 0
        demo_name = demonstration['name']
        demo_breakdown[demo_name] = {}
        for event in demonstration['events']:
            if event['type'] == 'set object pose':
                num_pose_events += 1
                obj_name = event['object_name']
                if obj_name in demo_breakdown[demo_name]:
                    demo_breakdown[demo_name][obj_name] += 1
                else:
                    demo_breakdown[demo_name][obj_name] = 0 # Ignore first one
            if event['type'] == 'spawn object':
                num_spawn_events += 1
        num_pose_events -= num_spawn_events
        num_frames = demonstration['frame_count']
        print '{} {} {} {}%'.format(demonstration['name'], num_pose_events, num_frames, 100.0 * num_pose_events/num_frames)
        for obj_name in demo_breakdown[demo_name]:
            print ' {} {}'.format(obj_name, demo_breakdown[demo_name][obj_name])

def get_number_of_skeleton_events(db):
    print 'Skeleton events'
    for demonstration in db.demonstrations.find():
        num_skeleton_events = 0
        for event in demonstration['events']:
            if event['frame_number'] == 0:
                continue
            if event['type'] == 'set skeleton state':
                num_skeleton_events += 1
        num_frames = demonstration['frame_count']
        print '{} {} {} {}%'.format(demonstration['name'], num_skeleton_events, num_frames, 100.0 * num_skeleton_events/num_frames)

def main():
    client = MongoClient()
    db = client.pbi
    get_number_of_set_pose_events(db)
    get_number_of_skeleton_events(db)

if __name__ == '__main__':
    main()
