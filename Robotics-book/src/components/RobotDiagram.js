import React from 'react';

// Simple robot diagram component to demonstrate MDX capability
const RobotDiagram = ({ type = "humanoid", title = "Robot Diagram" }) => {
  const renderDiagram = () => {
    switch(type) {
      case "humanoid":
        return (
          <div style={{ textAlign: 'center', fontFamily: 'monospace', padding: '20px' }}>
            <div style={{ fontSize: '1.2em', fontWeight: 'bold', marginBottom: '10px' }}>{title}</div>
            <div>    (o)   </div>
            <div>   /|\\   </div>
            <div>   / \\   </div>
            <div style={{ marginTop: '10px', fontSize: '0.9em' }}>
              Humanoid Robot Structure
            </div>
          </div>
        );
      case "arm":
        return (
          <div style={{ textAlign: 'center', fontFamily: 'monospace', padding: '20px' }}>
            <div style={{ fontSize: '1.2em', fontWeight: 'bold', marginBottom: '10px' }}>{title}</div>
            <div>   Shoulder   </div>
            <div>      |       </div>
            <div>   Elbow      </div>
            <div>      |       </div>
            <div>   Wrist      </div>
            <div>      |       </div>
            <div>   End-Effector</div>
          </div>
        );
      default:
        return (
          <div style={{ textAlign: 'center', padding: '20px' }}>
            <div style={{ fontSize: '1.2em', fontWeight: 'bold', marginBottom: '10px' }}>{title}</div>
            <div>Robot diagram type not recognized</div>
          </div>
        );
    }
  };

  return (
    <div style={{
      border: '1px solid #eaecef',
      borderRadius: '6px',
      margin: '20px 0',
      padding: '15px',
      backgroundColor: '#f8f9fa'
    }}>
      {renderDiagram()}
    </div>
  );
};

export default RobotDiagram;