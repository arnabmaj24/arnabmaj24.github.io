// Project groups, in display order. Used by ProjectGroups.astro and the Projects page's jump links.
export const GROUPS = [
  { key: 'work', title: 'Aston Dynamics', blurb: 'Products in customer beta: control, estimation, simulation and electronics.' },
  { key: 'team', title: 'Teams, research & competitions', blurb: 'University design teams, a research internship, FRC and a hackathon.' },
  { key: 'course', title: 'Course projects', blurb: '' },
] as const;
