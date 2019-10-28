class DestinationPlace
{
    public:
    double getXDestination() const;
    double getYDestination() const;
    double getZDestination() const;

    void setXDestination(const double x_destination);
    void setYDestination(const double y_destination);
    void setZDestination(const double y_destination);
    
    private:
    double m_destination_x;
    double m_destination_y;
    double m_destination_z;
};