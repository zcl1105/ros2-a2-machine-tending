"""Inspect workflow jobs and retrieve project logs/artifacts without leaking tokens."""
import argparse
import io
import json
from pathlib import Path
import sys
import urllib.request
import zipfile
from github_repo import api, credential

ROOT = Path(__file__).resolve().parents[1]


def download(token, url):
    request = urllib.request.Request(url, headers={'Authorization': 'Bearer '+token, 'User-Agent': 'a2-course-project'})
    with urllib.request.urlopen(request, timeout=90) as response:
        return response.read()


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser()
    parser.add_argument('--run-id', type=int, required=True)
    parser.add_argument('--logs', action='store_true')
    parser.add_argument('--download-artifacts', action='store_true')
    parser.add_argument('--repo', default='zcl1105/ros2-a2-machine-tending')
    args = parser.parse_args()
    token = credential()
    endpoint = f'/repos/{args.repo}/actions/runs/{args.run_id}'
    run = api(token, endpoint)
    print(json.dumps({key: run.get(key) for key in ('status', 'conclusion', 'html_url', 'head_sha')}))
    jobs = api(token, endpoint+'/jobs')
    failures = []
    for job in jobs['jobs']:
        print(json.dumps({'job': job['name'], 'steps': [(s['name'], s['conclusion']) for s in job['steps']]}))
        failures.extend(s['name'] for s in job['steps'] if s['conclusion'] == 'failure')
    if args.logs and run['status'] == 'completed':
        archive = zipfile.ZipFile(io.BytesIO(download(token, 'https://api.github.com'+endpoint+'/logs')))
        directory = ROOT/'results/ci'/str(args.run_id)
        directory.mkdir(parents=True, exist_ok=True)
        for name in archive.namelist():
            if not name.endswith('.txt'):
                continue
            content = archive.read(name).decode('utf-8-sig', 'replace')
            (directory/Path(name).name).write_text(content, encoding='utf-8')
            if any(failed.lower() in name.lower() for failed in failures):
                print(name+'\n'+content[-20000:])
        print('Logs: '+str(directory))
    if args.download_artifacts:
        for artifact in api(token, endpoint+'/artifacts')['artifacts']:
            if artifact['expired']:
                continue
            archive = zipfile.ZipFile(io.BytesIO(download(token, artifact['archive_download_url'])))
            directory = (ROOT/'results/downloads'/str(args.run_id)/artifact['name']).resolve()
            directory.mkdir(parents=True, exist_ok=True)
            for entry in archive.infolist():
                target = (directory/entry.filename).resolve()
                if not target.is_relative_to(directory):
                    raise ValueError('Unsafe artifact path')
                if entry.is_dir():
                    target.mkdir(parents=True, exist_ok=True)
                else:
                    target.parent.mkdir(parents=True, exist_ok=True)
                    target.write_bytes(archive.read(entry))
            print('Artifact: '+str(directory))


if __name__ == '__main__':
    main()
