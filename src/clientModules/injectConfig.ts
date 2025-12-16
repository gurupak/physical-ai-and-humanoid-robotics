import ExecutionEnvironment from '@docusaurus/ExecutionEnvironment';

if (ExecutionEnvironment.canUseDOM) {
  // Get the siteConfig from the Docusaurus context
  const script = document.querySelector('script[data-docusaurus-config]');
  if (script) {
    try {
      const config = JSON.parse(script.textContent || '{}');
      // Inject customFields into window for easy access
      (window as any).__DOCUSAURUS_CONFIG__ = config.customFields || {};
      console.log('Injected Docusaurus config:', (window as any).__DOCUSAURUS_CONFIG__);
    } catch (e) {
      console.error('Failed to parse Docusaurus config:', e);
    }
  }
}
