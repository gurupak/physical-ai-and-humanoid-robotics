import React from 'react';
import Link from '@docusaurus/Link';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import Layout from '@theme/Layout';
import HomepageHero from '@site/src/components/HomepageHero';
import HomepageCards from '@site/src/components/HomepageCards';

export default function Home(): JSX.Element {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout
      title={`Home`}
      description={siteConfig.tagline}>
      <HomepageHero />
      <main>
        <HomepageCards />
      </main>
    </Layout>
  );
}
