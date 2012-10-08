import unittest

try:
    from catkin.find_in_workspaces import find_in_workspaces, _get_valid_search_dirs
except ImportError as impe:
    raise ImportError(
        'Please adjust your pythonpath before running this test: %s' % str(impe))


class FindInWorkspaceTest(unittest.TestCase):

    def test_get_valid_search_dirs(self):
        self.assertEqual([], _get_valid_search_dirs([], None))
        self.assertEqual(['etc', 'include', 'libexec', 'share'],
                         _get_valid_search_dirs(None, 'foo'))
        self.assertEqual(['bin', 'etc', 'include', 'lib', 'share'],
                         _get_valid_search_dirs(None, None))
        self.assertEqual(['include', 'bin'],
                         _get_valid_search_dirs(['include', 'bin'], None))
        self.assertEqual(['include', 'etc'],
                         _get_valid_search_dirs(['include', 'etc'], 'foo'))

        self.assertRaises(ValueError, _get_valid_search_dirs, ['foo'], None)
        self.assertRaises(ValueError, _get_valid_search_dirs, ['bin'], 'foo')
        self.assertRaises(ValueError, _get_valid_search_dirs, ['libexec'], None)

   
