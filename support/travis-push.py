#!/usr/bin/env python

# NOTE: Call this from the root of the repo!
#       NOT from within the support direcotry.

from subprocess import CalledProcessError, check_call, check_output
from shlex import split as shlex_split
from os import environ, chdir, path
from string import replace
from re import match
from json import load as json_load, dump as json_dump


"""
Process:
    1) Pull the github.io repo
    2) Copy the files to the correct place in the repo
    3) Prepare the repo to be able to push
    4) Push
    5) ALWAYS - Cleanup


Env available:
    GIT_NAME
    GIT_EMAIL
    GH_TOKEN
"""

def load_version():
    """Retrieve the version from TinyG.h"""
    with open('firmware/tinyg/tinyg.h', 'r') as f:
        for line in f:
            m = match(r"^\s*#define\s+TINYG_FIRMWARE_BUILD\s+" \
                       "(?P<version>[0-9]+\.[0-9]+)", line)
            if m is not None:
                return m.group('version')
    

def pull_repo():
    """Pull the destination repo"""
    if not path.exists("github_io"):
        check_call(
            shlex_split('git clone ' \
                        'https://github.com/{SUBPATH}.git ' \
                        'github_io'.format(
                            SUBPATH=environ["DESTINATION_REPO_SUBPATH"]
                            )))

def copy_files():
    """Make a binary file, then copy the hex and binry to the destication"""

    hex_source = environ["SOURCE_HEX"]

    current_branch = check_output(
                        shlex_split(
                            'git symbolic-ref --short -q HEAD')).rstrip()

    if current_branch not in ['edge', 'master']:
        print "NOTICE: Only builds of edge or master get pushed to the web."
        exit(0)

    firmware_version = load_version()

    hex_destination = environ['DEST_FILE_TEMPLATE'].format(
                        BRANCH=current_branch,
                        VERSION=firmware_version,
                        EXT="hex"
                        )

    # Copy the hex
    check_call(
        shlex_split('cp {SOURCE_HEX} github_io/{DEST_HEX}'.format(
            SOURCE_HEX=hex_source,
            DEST_HEX=hex_destination
            )))

    # Update git
    chdir("github_io")

    check_call(
        shlex_split('git add {DEST_HEX}'.format(
            DEST_HEX=hex_destination
            )))

    # Update the JSON
    with open('tinyg-binaries.json', 'rb+') as f:
        # read JSON
        json_data = json_load(f)

        # Update the JSON
        new_version = {
                    "name" : "tinyg-{BRANCH}-{VERSION}".format(
                        BRANCH=current_branch,
                        VERSION=firmware_version
                    ),
                    "version" : firmware_version,
                    "branch" : current_branch
                }
                
        if new_version not in json_data["binaries"]:
            json_data["binaries"].append(
                new_version
            )
        else:
            print "NOTICE: This version is already in the repo. Exiting."
            exit(0)

        # rewind the file and trucate for writing
        f.seek(0,0)
        f.truncate()

        json_dump(json_data,
                  f,
                  separators=(',', ': '),
                  sort_keys=True,
                  indent=2)

    check_call(
        shlex_split('git add tinyg-binaries.json'))

    check_call(
        shlex_split('git commit -m ' \
            '"Added binaries for TinyG v{VERSION}"'.format(
             VERSION=firmware_version
            )))

    chdir("..")


def prepare_git():
    """Prepare git for pushing to the correct repo"""
    check_call(
        shlex_split('git config credential.helper '\
                    '"store --file=.git/credentials"'))
    with open('.git/credentials', 'w') as f:
        f.write("https://{GH_TOKEN}:@github.com".format(
            GH_TOKEN=environ["GH_TOKEN"]
            ))

def push_to_destination():
    """Push changes to the repo and tag it"""
    # Update git
    chdir("github_io")

    check_call(
        shlex_split('git push origin master'))

def main():
    """Prepare git and push the created binaries to the destination repo."""

    if "TRAVIS_PULL_REQUEST" in environ and environ['TRAVIS_PULL_REQUEST'] != "false":
        print "NOTICE: Pull request detected, unable to push built binary."
        exit(0)

    load_version()
    pull_repo()
    copy_files()
    prepare_git()
    push_to_destination()

if __name__ == '__main__':
    main()