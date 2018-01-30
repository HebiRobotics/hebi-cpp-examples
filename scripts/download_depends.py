import json
import os
import sys
import tarfile
import zipfile

if sys.version_info.major > 2:
  import urllib
  import urllib.request
else:
  import urllib2


################################
# Python Version Specific Functions

def GetURLRequest(url, headers):
  if sys.version_info.major > 2:
    return urllib.request.Request(url, headers=headers)
  else:
    return urllib2.Request(url, headers=headers)


def URLOpen(request):
  if sys.version_info.major > 2:
    try:
      return urllib.request.urlopen(request)
    except:
      print('Could not open URL: {0}'.format(request.get_full_url()))
      return None
  else:
    try:
      return urllib2.urlopen(request)
    except:
      print('Could not open URL: {0}'.format(request.get_full_url()))
      return None


################################
#

def ExtractAsset(assetDict, assetFile, output_dir):

  extract = assetDict['extract']
  extract_to = os.path.join(output_dir, extract['extract_to'])
  extract_folders = None

  if ('extract_folders' in extract):
    extract_folders = extract['extract_folders']

  ################
  # Is zip file
  if (os.path.splitext(assetFile)[-1] == '.zip'):
    archive = zipfile.ZipFile(assetFile)
    # Extract everything
    if (extract_folders == None):
      archive.extractall(extract_to)

    # Extract only specified folders
    else:
      for entry in archive.namelist():
        for folder in extract_folders:
          if (entry.startswith(folder)):
            archive.extract(entry, extract_to)
            break

  ################
  # Assuming that `tarfile` module can handle it
  elif(tarfile.is_tarfile(assetFile)):
    archive = tarfile.open(assetFile)
    # Extract everything
    if (extract_folders == None):
      archive.extractall(extract_to)

    # Extract only specified folders
    else:
      for entry in archive.getmembers():
        for folder in extract_folders:
          if (entry.name.startswith(folder)):
            archive.extract(entry, extract_to)
            break


def RunJobs(dependencies, output_dir):

  if (output_dir == None):
    output_dir = os.path.abspath(os.path.dirname(__file__))

  # Create output directory if it does not exist
  if not (os.path.exists(output_dir)):
    try:
      os.makedirs(output_dir)
    except:
      print('Could not make directory "{0}"'.format(output_dir))
      print('Exception: {0}'.format(sys.exc_info()[0]))

  for job in dependencies:

    target_asset = job['asset']

    # Download the asset
    asset_request_dict = dict()
    asset_request_dict['Accept'] = 'application/octet-stream'
    asset_file_name = target_asset['name'].format(\
      job['major_version'],\
      job['minor_version'],\
      job['revision_version']\
    )
    asset_url = job['asset_url'].format(asset_file_name)

    asset_request = GetURLRequest(asset_url, asset_request_dict)
    asset_response = URLOpen(asset_request)
    asset_content = asset_response.read()

    asset_output_file_name = target_asset['output_name'].format(\
      job['major_version'],\
      job['minor_version'],\
      job['revision_version']\
    )

    asset_output_file = os.path.join(output_dir, asset_output_file_name)

    asset_fd = open(asset_output_file, "wb")
    asset_fd.write(asset_content)
    asset_fd.close()

    # Extract this 
    if ('extract' in target_asset):
      ExtractAsset(target_asset, asset_output_file, output_dir)


################
# Run

# This will return a JSON array, so we can iterate over it below
dependencies = json.load(open(os.path.join(os.path.dirname(__file__), 'depends.json'), 'r'))

if (len(sys.argv) > 1):
  output_dir = sys.argv[1]
else:
  output_dir = None

RunJobs(dependencies, output_dir)
