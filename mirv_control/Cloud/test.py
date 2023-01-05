#!/usr/bin/env python3
import json
import requests

endpoint = "mirv-cloud-api-pre-prod.cloud.vtti.vt.edu"

def get_token():
    login_data = {"username": "jwiens@neaeraconsulting.com", "password":"energycameramood"}
    response = requests.post(f"https://{endpoint}/token", data=login_data, timeout=20)
    contents = json.loads(response.content.decode('utf-8'))
    return contents.get('access_token')

token = get_token()

print(token)
