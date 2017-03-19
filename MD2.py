""" The floorplan of the second floor of MD.

Author:
    Annaleah Ernst
"""

from math import pi

points = {'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U'}

locations = { 'A' : (-2.1336, -20.1168),
    'B' : (-1.2192, -20.1168),
    'C' : (-1.9, -16.764),
    'D' : (-1.2192, -16.764),
    'E' : (-1.9, -14.6304),
    'F' : (-1.2192, -14.6304),
    'G' : (-1.9, -9.4488),
    'H' : (-1.2192, -9.4488),
    'I' : (-1.9, -7.3152),
    'J' : (-1.2192, -7.3152),
    'K' : (-1.9, -2.1336),
    'L' : (-1.2192, -2.1336),
    'M' : (-1.9, 0.0),
    'N' : (-1.2192, 0.0),
    'O' : (-0.9144, 0.0),
    'P' : (-1.9, 5.1816),
    'Q' : (-1.2192, 5.1816),
    'R' : (-1.9, 7.3152),
    'S' : (-1.2192, 7.3152),
    'T' : (-1.9, 12.4968),
    'U' : (-1.2192, 12.4968)
    }

neighbors = {'A' : ['B'],
     'B' : ['A','D'],
     'C' : ['C'],
     'D' : ['B','C','F'],
     'E' : ['F'],
     'F' : ['D','E','H'],
     'G' : ['H'],
     'H' : ['F','G','J'],
     'I' : ['I'],
     'J' : ['H','I','L'],
     'K' : ['L'],
     'L' : ['J','K','N'],
     'M' : ['N'],
     'N' : ['L','M','O','Q'],
     'O' : ['N'],
     'P' : ['Q'],
     'Q' : ['N','P','S'],
     'R' : ['S'],
     'S' : ['Q','R','U'],
     'T' : ['U'],
     'U' : ['S','T']
    }

rooms = {253 : 'A',
     251 : 'C',
     249 : 'E',
     247 : 'G',
     245 : 'I',
     243 : 'K',
     241 : 'M',
     238 : 'O',
     239 : 'P',
     237 : 'R',
     235 : 'T'
    }

# of the form {tag_id : map_location}
landmarks = {1, 2}

# offset
#landmark_pos = {1 : (-2.4384, 11.8872),
#    2 : (-2.4384, 7.9248)
#    }

landmark_pos = {1 : (-2.4384, 12.4968),
    2 : (-2.4384, 7.3152)
    }

landmark_orient = {1 : pi / 2.0,
    2 : pi / 2.0
    }