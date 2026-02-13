#import "style.typ": main-title

#main-title([ASTRO:], [Autonomous Satellite Test & Robotics Operations])

#v(1em)

#grid(
  columns: 2,
  column-gutter: 0.5em,
  row-gutter: 1em,
  inset: (right: 0.25em),
  grid.vline(x: 0, position: end),
  emph[Project Information:], [],
  [Title], [ASTRO: Autonomous Satellite Test & Robotics Operations],
  [Team Name], [ROS2],
  [Date], datetime.today().display(),
  [],[],
  emph[Advisor Information:],[],
  [Name], [Dr. Christopher “Chrispy” Petersen],
  [Email], link("mailto:c.petersen1@ufl.edu"),
  [],[],
  emph[Team Roles:], [],
  [Project Manager], [Cannon Whitney],
  [SCRUM Master], [Dylan Long],
  [Backend Developer], [Caleb Jackson]
)