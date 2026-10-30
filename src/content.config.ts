import { defineCollection, z } from 'astro:content';
import { glob } from 'astro/loaders';

const projects = defineCollection({
  loader: glob({ pattern: '**/*.mdx', base: './src/content/projects' }),
  schema: z.object({
    title: z.string(),
    // One-line role/discipline tag shown under the title, e.g. "Controls & Embedded Systems"
    kicker: z.string(),
    // Short line for cards/index, 1 sentence
    tagline: z.string(),
    summary: z.string(),
    status: z.string(),
    year: z.string(),
    stack: z.array(z.string()),
    tags: z.array(z.string()),
    role: z.array(z.string()),
    links: z
      .array(
        z.object({
          label: z.string(),
          href: z.string(),
        })
      )
      .default([]),
    confidential: z.boolean().default(false),
    featured: z.boolean().default(false),
    // Drafts are left out of the build (pages, lists, links) until their text is checked.
    draft: z.boolean().default(false),
    // false keeps a project off the home page (it stays on the Projects page).
    home: z.boolean().default(true),
    // Optional button on the project's card, e.g. the sway demo on the brake controller.
    demo: z.object({ label: z.string(), href: z.string() }).optional(),
    // "full" gets the whole case-study template, "compact" gets a shorter one-page treatment
    size: z.enum(['full', 'compact']).default('full'),
    order: z.number().default(99),
    // Where the project sits on /projects: work (Aston), team (teams, research, competitions) or course.
    group: z.enum(['work', 'team', 'course']).default('team'),
    // Team size or setting, e.g. "Solo" or "3-person team". Shown in the project header.
    team: z.string().optional(),
    // Headline results shown on cards and at the top of the project page. Every value must be a confirmed fact.
    highlights: z
      .array(
        z.object({
          value: z.string(),
          label: z.string(),
        })
      )
      .default([]),
    // "contain" letterboxes portrait photos instead of cropping them to 16:9.
    coverFit: z.enum(['cover', 'contain']).default('cover'),
    coverLabel: z.string(),
    // Optional real media to replace the placeholder panel once assets exist.
    // Path relative to /public, e.g. "/media/projects/tvc-rocket/cover.mp4"
    coverSrc: z.string().optional(),
    coverKind: z.enum(['image', 'video']).default('image'),
  }),
});

export const collections = { projects };
