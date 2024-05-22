import { defineConfig } from 'astro/config';
import starlight from '@astrojs/starlight';

// https://astro.build/config
export default defineConfig({
	integrations: [
		starlight({
			title: 'Home',
			social: {
				github: 'https://gitlab.com/roar-gokart',
			},
			sidebar: [
				{
					label: 'Guides',
					autogenerate: { directory: 'Guide' },
				},
				{
					label: 'Documentation',
					autogenerate: { directory: 'Documentations' },
				},
			],
		}),
	],
});
