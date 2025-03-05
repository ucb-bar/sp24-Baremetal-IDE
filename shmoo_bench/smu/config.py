
class ShmooTest():

    def __init__(self, id: int, vector: bytes, expect: bytes):
        self.id = id
        self.vector = vector
        self.expect = expect

    def check_output(self, value: bytes):
        return self.expect == value

SHMOO_TESTS = [
    ShmooTest(
        0,
        b'1234',
        b'2345'
    ),
]
