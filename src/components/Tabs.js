import React from 'react';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

// Example implementation of a custom tabs component
// In practice, you would typically use Docusaurus's built-in Tabs component
const CustomTabs = ({ children, defaultValue, groupId }) => {
  return (
    <Tabs groupId={groupId} defaultValue={defaultValue}>
      {children}
    </Tabs>
  );
};

const CustomTabItem = ({ children, value, label }) => {
  return (
    <TabItem value={value} label={label}>
      {children}
    </TabItem>
  );
};

export { CustomTabs as Tabs, CustomTabItem as TabItem };