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
    // "full" gets the whole case-study template, "compact" gets a shorter one-page treatment
    size: z.enum(['full', 'compact']).default('full'),
    order: z.number().default(99),
    coverLabel: z.string(),
    // Optional real media to replace the placeholder panel once assets exist.
    // Path relative to /public, e.g. "/media/projects/tvc-rocket/cover.mp4"
    coverSrc: z.string().optional(),
    coverKind: z.enum(['image', 'video']).default('image'),
  }),
});

export const collections = { projects };
