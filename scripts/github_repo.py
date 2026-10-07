"""Private GitHub publication and compact CI status; never prints credentials."""
import argparse
import json
import os
from pathlib import Path
import re
import shutil
import subprocess
import sys
import urllib.error
import urllib.request

ROOT = Path(__file__).resolve().parents[1]


def git(*arguments, capture=False, input_text=None):
    executable = shutil.which('git')
    if not executable:
        raise RuntimeError('Git is unavailable')
    return subprocess.run([executable, *arguments], cwd=ROOT, input=input_text,
                          text=True, capture_output=capture, check=True,
                          env=dict(os.environ, GIT_TERMINAL_PROMPT='0', GCM_INTERACTIVE='Never'))


def credential():
    token = os.environ.get('GH_TOKEN') or os.environ.get('GITHUB_TOKEN')
    if token:
        return token
    try:
        result = git('credential', 'fill', capture=True, input_text='protocol=https\nhost=github.com\n\n')
        fields = dict(line.split('=', 1) for line in result.stdout.splitlines() if '=' in line)
        token = fields.get('password')
    except subprocess.CalledProcessError:
        token = None
    if not token:
        raise RuntimeError('No GitHub credential; sign in with Git Credential Manager / gh first')
    return token


def api(token, path, data=None):
    request = urllib.request.Request(
        'https://api.github.com'+path,
        data=json.dumps(data).encode() if data is not None else None,
        headers={'Authorization': 'Bearer '+token, 'Accept': 'application/vnd.github+json',
                 'X-GitHub-Api-Version': '2022-11-28', 'User-Agent': 'a2-course-project'})
    with urllib.request.urlopen(request, timeout=30) as response:
        return json.load(response)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--repo', default='ros2-a2-machine-tending')
    parser.add_argument('--check', action='store_true')
    parser.add_argument('--publish', action='store_true')
    parser.add_argument('--commit-message', default='Complete A2 machine tending simulation and course documentation')
    options = parser.parse_args()
    if not re.fullmatch(r'[A-Za-z0-9_.-]+', options.repo):
        raise ValueError('Invalid repository name')
    token = credential()
    user = api(token, '/user')
    owner = user['login']
    print(f'GitHub account: {owner}', flush=True)
    endpoint = f'/repos/{owner}/{options.repo}'
    try:
        repository = api(token, endpoint)
    except urllib.error.HTTPError as error:
        if error.code != 404:
            raise
        repository = None
    if options.publish:
        expected_url = f'https://github.com/{owner}/{options.repo}.git'
        local_git = (ROOT/'.git').exists()
        if repository is not None:
            if not local_git:
                raise RuntimeError('Repository already exists; refusing to connect an unrelated local project')
            remote = git('remote', 'get-url', 'origin', capture=True).stdout.strip()
            if remote != expected_url:
                raise RuntimeError('Existing origin differs; refusing to replace it')
        else:
            repository = api(token, '/user/repos', {
                'name': options.repo, 'private': True, 'auto_init': False,
                'description': 'ROS 2 Humble A2 machine tending course demo: SCARA simulation, faults, evidence and video guide'})
        if not local_git:
            git('init', '-b', 'main')
            git('remote', 'add', 'origin', expected_url)
        git('config', 'user.name', user.get('name') or owner)
        git('config', 'user.email', f"{user['id']}+{owner}@users.noreply.github.com")
        git('add', '.')
        changed = git('diff', '--cached', '--name-only', capture=True).stdout.strip()
        if changed:
            git('commit', '-m', options.commit_message)
        git('push', '-u', 'origin', 'main')
        print('Published repository: '+repository['html_url'], flush=True)
    if repository:
        print('Repository: '+repository['html_url'], flush=True)
        runs = api(token, endpoint+'/actions/runs?per_page=3')
        for run in runs.get('workflow_runs', []):
            print(json.dumps({key: run.get(key) for key in
                              ('id', 'head_sha', 'status', 'conclusion', 'html_url')}, ensure_ascii=False))
    else:
        print('Repository does not yet exist', flush=True)


if __name__ == '__main__':
    try:
        main()
    except (RuntimeError, ValueError, urllib.error.HTTPError, subprocess.CalledProcessError) as error:
        print('GitHub operation failed: '+str(error), file=sys.stderr)
        sys.exit(1)
