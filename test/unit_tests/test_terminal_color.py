import unittest

try:
    from catkin.terminal_color import fmt, sanitize
except ImportError as e:
    raise ImportError(
        'Please adjust your pythonpath before running this test: %s' % str(e)
    )


class TerminalColorTest(unittest.TestCase):

    def test_terminal_colors(self):
        test = '@_This is underlined@|'
        rslt = '\033[4mThis is underlined\033[0m\033[0m'
        assert fmt(test) == rslt
        test = 'This has bad stuff @! @/ @_ @| OK!'
        test = sanitize(test)
        rslt = 'This has bad stuff @! @/ @_ @| OK!\033[0m'
        assert fmt(test) == rslt
        test = u'\u2018@'
        test = sanitize(test)
        rslt = u'\u2018@@'
        assert test == rslt
