"""A request the hub will not carry out. str() is a sentence saying why, with the limits; nothing was changed."""


class Refused(Exception):
    def __init__(self, message: str, status: int = 400):
        super().__init__(message)
        self.status = status
