#!/usr/bin/env python

# Imports
import sys
import json

def process_profile_json(file_path):
    with open(file_path, 'r') as f:
        data = json.load(f)
    
    profile_results = {}

    for entry in data["traceEvents"]:
        if entry["name"] in profile_results:
            profile_results[entry["name"]].append(entry["dur"])
        else:
            profile_results[entry["name"]] = [entry["dur"]]
    
    for key in profile_results.keys():
        print("The average time for " + key + " is: ", str(sum(profile_results[key])/len(profile_results[key])))
    
if __name__ == '__main__':
    print(len(sys.argv))
    if len(sys.argv) < 2:
        print("Error: please pass a file path for the profile JSON")
    else:
        process_profile_json(sys.argv[1])