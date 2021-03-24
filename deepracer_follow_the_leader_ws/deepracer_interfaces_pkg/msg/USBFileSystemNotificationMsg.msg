# Custom message with the details of file/folder found in the USB drive
# and corresponding subscriber callback details.
#
# This broadcast message is used to publish the file/folder information and
# its callback information that was set when subscribing to be notified if
# the file/folder is found in the USB drive.
string path             # Path of the file in the temp mount system.
string file_name        # File/folder name found.
string callback_name    # Callback name that was sent while subscribing
                        # to be notified.
string node_name        # Node name to manage dereferencing the mount point
                        # after the processing of the file/folder is completed.