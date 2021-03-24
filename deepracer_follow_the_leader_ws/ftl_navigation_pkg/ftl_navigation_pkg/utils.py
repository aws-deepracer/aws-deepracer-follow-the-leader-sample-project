#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################

import threading

#########################################################################################
# Double Buffer.


class DoubleBuffer():
    """Object type which helps to thread-safely read and write from the buffer.
    """
    def __init__(self, clear_data_on_get=True):
        """Create a DoubleBuffer object.

        Args:
            clear_data_on_get (bool, optional): Flag to clear data from the queue after its read.
                                                Defaults to True.
        """
        self.read_buffer = None
        self.write_buffer = None
        self.clear_data_on_get = clear_data_on_get
        self.cv = threading.Condition()

    def clear(self):
        """Helper method to clear the buffer.
        """
        with self.cv:
            self.read_buffer = None
            self.write_buffer = None

    def put(self, data):
        """Helper method to safely store the data in the buffer.

        Args:
            data (Any): The object that is to be stored.
        """
        with self.cv:
            self.write_buffer = data
            self.write_buffer, self.read_buffer = self.read_buffer, self.write_buffer
            self.cv.notify()

    def get(self, block=True):
        """Helper method to safely read the data from the buffer.

        Args:
            block (bool, optional): Flag set to wait for the new data if read_buffer is empty.
                                    Defaults to True.

        Raises:
            DoubleBuffer.Empty: Exception if returning empty read buffer without waiting for
                                new data.

        Returns:
            Any: Data stored in the buffer.
        """
        with self.cv:
            if not block:
                if self.read_buffer is None:
                    raise DoubleBuffer.Empty
            else:
                while self.read_buffer is None:
                    self.cv.wait()
            data = self.read_buffer
            if self.clear_data_on_get:
                self.read_buffer = None
            return data

    def get_nowait(self):
        """Wrapper method to return the data stored in buffer without waiting for new data.

        Returns:
            Any: Data stored in the buffer.
        """
        return self.get(block=False)

    def is_empty(self):
        """Wrapper method to check if buffer has new data.

        Returns:
            (bool): Whether buffer is empty or not
        """
        with self.cv:
            if self.read_buffer is None:
                return True
            else:
                return False

    class Empty(Exception):
        """Custom Exception to identify if an attempt was made to read from an empty buffer.

        Args:
            Exception (Exception): Inherits from Exception class.
        """
        pass
