# Custom message with software update progress details.
#
# This message is used to send the software update progress
# percentage and the corresponding stage in the update process.
float32 update_pct  # [0-100] value to indicate the software update
                    # progress percentage 
string status       # Value indicating the stage of the update process.
                    # Ex: checking, downloading, installing, etc,.